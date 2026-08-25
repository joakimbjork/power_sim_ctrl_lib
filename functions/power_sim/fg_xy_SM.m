function [ fg, u] = fg_xy_SM(T,xy0,PAR)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-22
%% System info
% ws=PAR.ws; %Nominal frequency
ng=PAR.ng; %Number of generators
nbus=PAR.nbus; %Number of buses
nx = PAR.nx;
ns = PAR.ns;
Gbus=PAR.Gbus; %Index of generator buses

PL0=PAR.PL0; %Initial active loads
QL0=PAR.QL0; %Initial reactive loads
U0=PAR.U0; %Initial bus voltage amplitudes
ANG0 = PAR.ANG0; %Initial bus voltage phase angles

%% Generator info
M=PAR.M; %Generator innertia constant
D=PAR.D; %Generator damping constant

XDP=PAR.XDP; %d-axis transient reactance
% BDP = 1./XDP; %Generator transient susceptance(1./XDP)
XD=PAR.XD; %d-axis synchronous reactance
TDOP=PAR.TDOP; %d-axis transient open-circuit time constant

if ns <= 3
    XQP = XDP; % Transient saliency is neglected
    XQ = XQP; % No rotor windings on q-axis
elseif ns >= 4
    XQP = PAR.XQP; %q-axis transient reactance
    XQ = PAR.XQ; %q-axis synchronous reactance
    TQOP = PAR.TQOP; %q-axis transient open-circuit time constant
    %     if ns == 6
    %         XDPP = PAR.XDPP; %d-axis subtransient reactance
    %         TDOPP = PAR.TDOPP; %d-axis subtransient open-circuit time constant
    %         XQPP = PAR.XQPP; %q-axis subtransient reactance
    %         TQOPP = PAR.TQOPP; %q-axis subtransient open-circuit time constant
    %         XL = PAR.XL; % Stator leakage reactance
    %     end
end

%% States and algebraic variables
x0=xy0(1:nx); %Current state variable values
y0=xy0(nx+1:nx+2*nbus); %Current algebraic variable values
DELTA=x0(1:ng);
OMEGA=x0(ng+1:2*ng);
if ns >= 3
    EQP=x0(2*ng+1:3*ng);
else
    EQP = PAR.EQP;
end

if ns >= 4
    EDP=x0(3*ng+1:4*ng);
else
    EDP = PAR.EDP;
end

% if ns == 6
%     F1D = x0(4*ng+1:5*ng);
%     F2Q = x0(5*ng+1:6*ng);
% end

ANG=y0(1:nbus);
VOLT=y0(nbus+1:2*nbus);

%% Voltage pertubation test
if isfield(PAR,'VOLT_pert')
    bus = PAR.VOLT_pert.bus;
    VOLT(bus) = U0(bus)+PAR.VOLT_pert.u;
    % ANG(bus) = ANG0(bus); % Also holding the phase angle
end

ANGG = ANG(Gbus);
UG = VOLT(Gbus);
UG0 = U0(Gbus);
ANGG0 = ANG0(Gbus);

f_xy = zeros(nx,1);
u = [];



%% Line flow
if nbus == 1
    S_line = 0;
else
    V_complex = VOLT.*exp(1j*ANG);
    S_line = diag(V_complex)*conj(PAR.YBUS*V_complex);
end

%% Power injections at machine terminals
% Udq = UG.*exp(1j*(ANGG-DELTA+pi/2)); = 1j*UG.*exp(1j*(ANGG-DELTA))
% Ud = real(Udq);
% Uq = imag(Udq);
Ud = -UG.*sin(ANGG-DELTA);
Uq = UG.*cos(ANGG-DELTA);
Iq = (Ud-EDP)./XQP;
Id = -(Uq-EQP)./XDP;

% Alternative 1: Calculation of power injections
PE = Id.*Ud + Iq.*Uq;
QE = Id.*Uq - Iq.*Ud;
% PE = EDP.*Id + EQP.*Iq + (XQP-XDP).*Id.*Iq;
% QE = EQP.*Id - EDP.*Iq - XDP.*Id.^2 - XQP.*Iq.^2;

% Alternative 2: Only for one-axis or classic model
% PE=EQP.*UG.*sin(DELTA-ANGG)./XDP; %Active power output of generators
% QE=-(UG.^2-UG.*EQP.*cos(ANGG-DELTA))./XDP; %Rective power injection at generator bus

%% PODP (active power control)
% The PODP controller calculates an active power injection at a network
% node. At converter nodes, the PODP signal P_PODP is the reference signal
% to the coverter. At non converter buses it is an ideal controllable
% power injection.
if isfield(PAR,'PODP')
    dy = zeros(nbus,1);
    if isfield(PAR.PODP,'input_func')
        dy = PAR.PODP.input_func(T,dy);
        if isfield(PAR.PODP,'input_func_filter')
            [f_xy(PAR.PODP.input_func_filter.idx_nx), dy, ~] = ss_object(0, PAR.PODP.input_func_filter, x0, dy, zeros(nbus,1)); 
            dy_PODP = dy;
        end
    end
    if any(PAR.PODP.CL == 1)
        cl = PAR.PODP.CL;
    else
        cl = 0;
    end
    if isfield(PAR.PODP,'lin_fg_xy_input')% Linearization input 
        dy  = dy + PAR.PODP.lin_fg_xy_input;
    end
      
    [f_xy(PAR.PODP.PLL.idx_nx), FREQ_PODP, ~] = ss_object(0, PAR.PODP.PLL, x0, ANG-ANG0, zeros(nbus,1)); % [rad] --> [Hz]
    if isfield(PAR,'CONV') 
        if PAR.CONV.type >= 2% Use internal frequency from GFM or GFL controller           
            [~,~,C1,~] = ssdata(PAR.CONV.sync_ctrl.sys); % 
            x1 = x0(PAR.CONV.sync_ctrl.idx_nx); % pick out states
            OMEGAC = C1*x1;% + PAR.CONV.P0;
            FREQ_PODP(PAR.CONV.Cbus) = OMEGAC/(2*pi); %[rad/s] --> [Hz]
        end
    end

    if PAR.PODP.type == 1 %
        [f_xy(PAR.PODP.idx_nx), P_PODP, ~] = ss_object(0, PAR.PODP, x0, (-FREQ_PODP).*cl+dy, zeros(nbus,1));
    elseif PAR.PODP.type == 2 % PODP controller with exponential gain block
        [f_xy(PAR.PODP.K1.idx_nx), P_1, ~] = ss_object(0, PAR.PODP.K1, x0, -FREQ_PODP.*cl+dy, zeros(nbus,1));
        [f_xy(PAR.PODP.K2.idx_nx), P_2, ~] = ss_object(0, PAR.PODP.K2, x0, P_1, zeros(nbus,1));
        % Exponential funtion
        P_3 = sign(P_2).*abs(P_2).^PAR.PODP.A;
        [f_xy(PAR.PODP.idx_nx), P_PODP, ~] = ss_object(0, PAR.PODP, x0, P_3, zeros(nbus,1)); % main control block
    end

    if isfield(PAR.PODP,'bypass_input_func')
        P_PODP = P_PODP + PAR.PODP.bypass_input_func(T,dy);
    end
    if isfield(PAR.PODP,'bypass_lin_fg_xy_input')% Linearization input 
        P_PODP = P_PODP + PAR.PODP.bypass_lin_fg_xy_input;
    end
else
    FREQ_PODP = zeros(nbus,1);
    P_PODP = zeros(nbus,1);
end
u_P_PODP = P_PODP;
    



%% PODQ (reactive power control)
% The PODQ controller calculates an reactive power injection at a network
% node. At converter nodes, the PODQ signal Q_PODQ is the reference signal
% to the coverter. At non converter buses it is an ideal controllable
% power injection.
if isfield(PAR,'PODQ')
    dy = zeros(nbus,1);
    if isfield(PAR.PODQ,'input_func')
        dy = PAR.PODQ.input_func(T,dy);
    end
    if isfield(PAR.PODQ,'lin_fg_xy_input')% Linearization input 
        dy  = dy + PAR.PODQ.lin_fg_xy_input;
    end
    if any(PAR.PODQ.CL == 1)
        cl = PAR.PODQ.CL;
    else
        cl = 0;
    end
    if PAR.PODQ.type == 1 %
        [f_xy(PAR.PODQ.idx_nx), Q_PODQ, ~] = ss_object(0, PAR.PODQ, x0, (U0-VOLT).*cl+dy, zeros(nbus,1));
    elseif PAR.PODQ.type == 2 %
        [f_xy(PAR.PODQ.idx_nx), Q_PODQ, ~] = ss_object(0, PAR.PODQ, x0, (ANG0-ANG).*cl+dy, zeros(nbus,1));
    elseif PAR.PODQ.type == 3 % Dead band controller [absolute dead band]
        [f_xy(PAR.PODQ.K1.idx_nx), Q_1, ~] = ss_object(0, PAR.PODQ.K1, x0, (U0-VOLT).*cl+dy, zeros(nbus,1));
        [f_xy(PAR.PODQ.K2.idx_nx), Q_2, ~] = ss_object(0, PAR.PODQ.K2, x0, (U0-VOLT).*cl+dy, zeros(nbus,1));
        if any(cl == 1)
            [f_xy(PAR.PODQ.Kaux.idx_nx), Q_aux, ~] = ss_object(0, PAR.PODQ.Kaux, x0, -VOLT+dy, zeros(nbus,1)); % Dead band controller. Initial value should be within dead band limit
        else
            [f_xy(PAR.PODQ.Kaux.idx_nx), Q_aux, ~] = ss_object(0, PAR.PODQ.Kaux, x0, -U0+dy, zeros(nbus,1)); % Dead band controller. Initial value should be within dead band limit
        end
        Q_PODQ_ref = Q_1+Q_2+Q_aux;
        [f_xy(PAR.PODQ.idx_nx), Q_PODQ, ~] = ss_object(0, PAR.PODQ, x0, Q_PODQ_ref, zeros(nbus,1));
    elseif PAR.PODQ.type == 4 % Dead band controller [relative dead band]
        [f_xy(PAR.PODQ.K1.idx_nx), Q_1, ~] = ss_object(0, PAR.PODQ.K1, x0, (U0-VOLT).*cl+dy, zeros(nbus,1));
        [f_xy(PAR.PODQ.K2.idx_nx), Q_2, ~] = ss_object(0, PAR.PODQ.K2, x0, (U0-VOLT).*cl+dy, zeros(nbus,1));
        [f_xy(PAR.PODQ.COM.idx_nx), Q_COM, ~] = ss_object(0, PAR.PODQ.COM, x0, Q_1+Q_2, zeros(nbus,1));
        [f_xy(PAR.PODQ.Kaux.idx_nx), Q_aux, ~] = ss_object(0, PAR.PODQ.Kaux, x0, (U0-VOLT).*cl+dy, zeros(nbus,1));
        Q_PODQ_ref = Q_COM+Q_aux;
        [f_xy(PAR.PODQ.idx_nx), Q_PODQ, ~] = ss_object(0, PAR.PODQ, x0, Q_PODQ_ref, zeros(nbus,1));
    elseif PAR.PODQ.type == 5 % PODQ and voltage controller, coupled with an outer loop Q-controller (Needs converter object for Q feedback)
        [f_xy(PAR.PODQ.K1.idx_nx), Q_1, ~] = ss_object(0, PAR.PODQ.K1, x0, (U0-VOLT).*cl+dy, zeros(nbus,1)); % PODQ controller (inner loop)
        [f_xy(PAR.PODQ.K2.idx_nx), Q_2, ~] = ss_object(0, PAR.PODQ.K2, x0, (U0-VOLT).*cl+dy, zeros(nbus,1)); % Voltage controller (outer loop)
        
        [~,~,C1,~] = ssdata(PAR.PODQ.sys); % 
        x1 = x0(PAR.PODQ.idx_nx); % pick out states
        Q_meas_diff = C1*x1;

        if isfield(PAR,'CONV') % Use Q measurement from converter object
            Cbus = PAR.CONV.Cbus;
            [~,~,C1,~] = ssdata(PAR.CONV.Q_meas.sys); % 
            x1 = x0(PAR.CONV.Q_meas.idx_nx); % pick out states
            Q_meas_diff(Cbus) = C1*x1; % + PAR.QC; % Removing PAR.QC since we want the diff
        end

        [f_xy(PAR.PODQ.KQ.idx_nx), Q_3, ~] = ss_object(0, PAR.PODQ.KQ, x0, Q_2 - Q_meas_diff, zeros(nbus,1)); % Voltage controller (outer loop)
        [f_xy(PAR.PODQ.idx_nx), Q_PODQ, ~] = ss_object(0, PAR.PODQ, x0, Q_1 + Q_3, zeros(nbus,1)); % Sumation or inverter model (can be = 1 for conv objects)
    end
else
    Q_PODQ = zeros(nbus,1);
end
u_Q_PODQ = Q_PODQ;

%% Converter
Pinject = PAR.P0;
Qinject = PAR.Q0;

if isfield(PAR,'CONV')
    Cbus = PAR.CONV.Cbus;
    PC = PAR.PC;
    QC = PAR.QC;
    % PC0 = PC;
    % QC0 = QC;
    nc = length(Cbus);
    dy_P = zeros(nc,1);
    dy_Q = zeros(nc,1);
    if isfield(PAR.CONV,'P_input_func')
        dy_P = PAR.CONV.P_input_func(T);
    end
    if isfield(PAR.CONV,'Q_input_func')
        dy_Q = PAR.CONV.Q_input_func(T);
    end
    dy = [dy_P;dy_Q];
    PC = dy_P + PC;
    QC = dy_Q + QC;

    if isfield(PAR.CONV,'Pref_lin_fg_xy_input')
        PC = PC + PAR.CONV.Pref_lin_fg_xy_input;
    end

    if isfield(PAR.CONV,'Storage') % Change of charge estimator used by the frequency controller
        [~,~,C1,~] = ssdata(PAR.CONV.P_meas.sys); % 
        x1 = x0(PAR.CONV.P_meas.idx_nx); % pick out states
        P_meas = C1*x1 + PAR.PC;
        Pset = PC; % + PAR.CONV.P0; 
        P_est = P_meas - Pset; % Power change caused by change of external power set point (PAR.CONV.P0 + dy_P) should not affect the internal recharging controller
        [f_xy(PAR.CONV.Storage.idx_nx), Echarge, ~] = ss_object(0, PAR.CONV.Storage, x0, P_est, zeros(nc,1));
        
        
        Eref = zeros(nc,1); % Default value for the frequency controller (real charge control is done in top-level power control)
        [f_xy(PAR.CONV.Storage.Charge_ctrl.idx_nx), dP_charge, ~] = ss_object(0, PAR.CONV.Storage.Charge_ctrl, x0, Eref-Echarge, zeros(nc,1));
        
        % PI control with POD-P gain compensation. The P-control and/or the
        % PI-control should have a low pass filter with a time constant
        % somewhere below the synchronizing bandwidth.
        % Kp = 5;
        % [f_xy(PAR.CONV.Storage.Charge_ctrl.idx_nx), dP_charge, ~] = ss_object(0, PAR.CONV.Storage.Charge_ctrl, x0, Eref-Echarge-Kp*P_est, zeros(nc,1)); % PI control
        % Kcomp = eye(nc) + PAR.CONV.Storage.Charge_ctrl.sys.D*Kp;
        % P_PODP(Cbus) = Kcomp*P_PODP(Cbus);
    

        dP = P_PODP(Cbus)+dP_charge;

        % Energy management (hard coded example)      
        % Elim = 2.0; % Energy charge limit
        % k = 5/Elim; % Transitional gain. 
        % Emax = ones(nc,1)*(Elim-1/k); % Start limiting output before reaching Elim, to make a smooth transition.
        % Pmax = max(zeros(nc,1),  ones(nc,1)+k*(Emax-Echarge));
        % Emin = -Emax; % Assume symmetric limitation
        % Pmin = min(zeros(nc,1), -ones(nc,1)+k*(Emin-Echarge));
        % dP = min(Pmax,dP);
        % dP = max(Pmin,dP);
    elseif isfield(PAR.CONV,'HVDC_wind')
        [~,~,C1,~] = ssdata(PAR.CONV.HVDC_wind.Gdc.sys); %
        x1 = x0(PAR.CONV.HVDC_wind.Gdc.idx_nx); % pick out states
        Vdc0 = PAR.CONV.HVDC_wind.Vdc0;
        Vdc = C1*x1+Vdc0;
        [f_xy(PAR.CONV.HVDC_wind.K2.idx_nx), HVDC_wind_dP_K2] = ss_object(0, PAR.CONV.HVDC_wind.K2, x0, Vdc-Vdc0, zeros(nc,1)); % Need to run this again inside HVDC_wind_object
        
        dP = P_PODP(Cbus) + HVDC_wind_dP_K2;
        % Vdc_REF = 1*ones(nc,1);
        % [f_xy(PAR.CONV.HVDC_wind.idx_nx),dP_dc] = HVDC_wind_object(0, PAR.CONV.HVDC_wind, x0, dP, Vdc_REF); % <--- FIX

    else
        dP_charge = zeros(nc,1);
        dP = P_PODP(Cbus)+dP_charge;
    end

    UC = VOLT(Cbus);
    ANGC = ANG(Cbus);
    
    

    PQ_CONV_REF = [PC+dP; QC+Q_PODQ(Cbus)];
    
    [f_xy(PAR.CONV.idx_nx),y_CONV] = CONV_object(0, PAR.CONV, x0, [UC; ANGC], PQ_CONV_REF);
  
    % PC = y_CONV.Pe; 
    % PQ = y_CONV.Qe;
    % PQ measurements are updated inside converter object
    % [f_xy(PAR.CONV.P_meas.idx_nx), P_meas] = ss_object(0,PAR.CONV.P_meas, x0, PC-PC0, PC0); 
    % [f_xy(PAR.CONV.Q_meas.idx_nx), Q_meas] = ss_object(0, PAR.CONV.Q_meas, x0, QC-QC0, QC0);

    Pinject(Cbus) = y_CONV.Pe; 
    Qinject(Cbus) = y_CONV.Qe;

    % DELTA_CONV = DW_CONV(1:nc,1); OMEGA_CONV = DW_CONV(nc+(1:nc));
    % Ed_CONV = Edq_CONV(1:nc,1); Eq_CONV = Edq_CONV(nc+(1:nc));
    % Id_CONV = Idq_CONV(1:nc,1); Iq_CONV = Idq_CONV(nc+(1:nc));

    u_P_PODP = P_PODP;
    u_Q_PODQ = Q_PODQ;
    P_PODP(Cbus) = 0; % P_PODP control at converter buses will only act on the converter object
    Q_PODQ(Cbus) = 0; % Q_PODQ control at converter buses will only act on the converter object

    if isfield(PAR.CONV,'HVDC_wind')
        P2 = y_CONV.Pe;         
        Vdc_REF = 1*ones(nc,1);
        [f_xy(PAR.CONV.HVDC_wind.idx_nx),y_HVDC_wind] = HVDC_wind_object(0, PAR.CONV.HVDC_wind, x0, P2, Vdc_REF);
        y_HVDC_wind.dP_K2 =  HVDC_wind_dP_K2; % Save for output
    end
end

%% Loads and power injections
mp=PAR.mp; %Active load characteristic
mq=PAR.mq; %Reactive load characteristic
PL= -Pinject + (PL0./(U0.^mp)).*(VOLT.^(mp)) - P_PODP; %Active power load
QL= -Qinject +(QL0./(U0.^mq)).*(VOLT.^(mq)) - Q_PODQ; %Reactive power load

%% Exogenous constant power load changes
if isfield(PAR,'dPL')
    PL = PL + PAR.dPL; % Active power injection (constant power load)
end
if isfield(PAR,'dPL_func')
    PL = PL + PAR.dPL_func(T);
end
if isfield(PAR,'dQL')
    QL = QL + PAR.dQL; % Reactive power injection (constant power load)
end
if isfield(PAR,'dQL_func')
    QL = QL + PAR.dQL_func(T);
end

%% PMU
if isfield(PAR,'PMU')
    if PAR.PMU.type == 1 % Linear model
        [f_xy(PAR.PMU.idx_nx), FREQ, ~] = ss_object(0, PAR.PMU, x0, ANG-ANG0, zeros(nbus,1));
    end
end

%% Wind turbine
if isfield(PAR,'WIND')
    if PAR.WIND.type == 1 % Linear model
        [f_xy(PAR.WIND.FFR.idx_nx), dPwind_ref, ~] = ss_object(0, PAR.WIND.FFR, x0, -FREQ, zeros(nbus,1));
        [f_xy(PAR.WIND.idx_nx), dPwind, ~] = ss_object(0, PAR.WIND, x0, dPwind_ref, zeros(nbus,1));
        [f_xy(PAR.WIND.SPEED.idx_nx), d_wind_rot_speed, ~] = ss_object(0, PAR.WIND.SPEED, x0, dPwind_ref, zeros(nbus,1));
    end
    PL = PL - dPwind;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Algebraic equations, power balance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PP =  real(S_line) + PL;
PP(Gbus) = PP(Gbus) - PE;
QQ =  imag(S_line) + QL;
QQ(Gbus) = QQ(Gbus) - QE;
g_xy=[PP;
    QQ];

%% Governor
if isfield(PAR,'GOV')
    PM0 = PAR.PM; %Generator mechanical input power
    if any(PAR.GOV.CL == 1)
        cl = PAR.GOV.CL;
    else
        cl = 0;
    end
    dy = zeros(ng,1);
    if isfield(PAR.GOV,'input_func')
        dy = PAR.GOV.input_func(T,dy);
    end
    if isfield(PAR.GOV,'lin_fg_xy_input')% Linearization input 
        dy  = dy + PAR.GOV.lin_fg_xy_input;
    end
    [f_xy(PAR.GOV.idx_nx), PM, ~, PM_ramp_error] = ss_object(0, PAR.GOV, x0, dy-OMEGA.*cl, PM0);
else
    PM = PAR.PM; %Generator mechanical input power
end

%% Exciter
EF0 = PAR.UREF; %Generator reference voltage
if isfield(PAR,'EXC')
    if isfield(PAR,'PSS')
        if PAR.PSS.CL == 0
            cl = 0;
        else
            cl = 1;
        end
        dy = zeros(ng,1);
        if isfield(PAR.PSS,'input_func')
            dy = PAR.PSS.input_func(T,dy);
        end
        if PAR.PSS.type == 1 % Rotor speed feedback
            [f_xy(PAR.PSS.idx_nx), u_PSS, ~] = ss_object(0, PAR.PSS, x0, OMEGA.*cl + dy, zeros(ng,1));
        elseif PAR.PSS.type == 2 % Terminal voltage angle feedback
            [f_xy(PAR.PSS.idx_nx), u_PSS, ~] = ss_object(0, PAR.PSS, x0, (ANGG-ANGG0).*cl + dy, zeros(ng,1));
        elseif PAR.PSS.type == 3 % Electric power feedback
            [f_xy(PAR.PSS.idx_nx), u_PSS, ~] = ss_object(0, PAR.PSS, x0, (PE-PAR.PE0).*cl + dy, zeros(ng,1));
        elseif PAR.PSS.type == 4 % Compensated frequency
            % This is a simple test implementation. Possible issue with phase jump in
            % "angle(EQP_comp)", may have to be fixed. The output range of
            % "angle" is: -pi<x<pi, so a phase jump issue is likely to occur
            % if power changes direction, i.e. if we go from generator to
            % motor drive. 
            XQP_est = XQP*1.0;

            I_conj = (PE - 1j*QE)./UG;
            EQP_comp = UG+1j*XQP_est.*I_conj;
            ANGG_comp = ANGG + angle(EQP_comp); % Compensated freqeuncy

            I_conj = (PAR.PE0 - 1j*PAR.QE0)./UG0;
            EQP_comp = UG0+1j*XQP_est.*I_conj;
            ANGG0_comp = ANGG0 + angle(EQP_comp); % Initial compensated frequency

            [f_xy(PAR.PSS.idx_nx), u_PSS, ~] = ss_object(0, PAR.PSS, x0, (ANGG_comp-ANGG0_comp).*cl + dy, zeros(ng,1));
        end
    else
        u_PSS = zeros(ng,1);
    end

    if PAR.EXC.CL == 0
        cl = 0;
    else
        cl = 1;
    end
    dy = zeros(ng,1);
    if isfield(PAR.EXC,'input_func')
        dy = PAR.EXC.input_func(T,dy);
    elseif isfield(PAR,'u_UREF') % Used by lin_fg_xy
        dy = PAR.u_UREF;
    end
    % if isfield(PAR,'AVR') % Standard is to have AVR togehter with EXC
    %     [f_xy(PAR.AVR.idx_nx), u_AVR, ~] = ss_object(0, PAR.AVR, x0, UG0-UG, zeros(ng,1));
    % else
    %     u_AVR = zeros(ng,1);
    % end

    u_EXC = (UG0-UG).*cl + u_PSS + dy; % + u_AVR
    [f_xy(PAR.EXC.idx_nx), EF, EF_ref] = ss_object(0, PAR.EXC, x0, u_EXC, EF0);
   
else
    dy = zeros(ng,1);
    if isfield(PAR,'u_UREF') % Used by lin_fg_xy
        dy = PAR.u_UREF-EF0;
    end
    EF = EF0 + dy;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Differential equations, generator dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f_xy(1:2*ng)=[
    OMEGA; % dDELTA/dt
    (PM-PE-D.*OMEGA)./M % dOMEGA/dt
    ];

if ns >= 3
    f_xy(2*ng+1:3*ng)=[
        (-EQP - (XD-XDP).*Id + EF)./TDOP % dEQP/dt
        ];
end

if ns >= 4
    f_xy(3*ng+1:4*ng)=[
        (-EDP + (XQ-XQP).*Iq )./TQOP % dEDP/dt
        ];
end

if ns == 6
    F1D = x0(4*ng+1:5*ng);
    c = (XDP-PAR.XDPP)./((XDP-PAR.XL).^2);
    z = (F1D + (XDP-PAR.XL).*Id - EQP);

    f_xy(2*ng+1:3*ng) = (-EQP - (XD-XDP).*(Id - c.*z) + EF)./TDOP; % dEQP/dt
    f_xy(4*ng+1:5*ng) = -z./PAR.TDOPP; % dF1D/dt

    F2Q = x0(5*ng+1:6*ng);
    c = (XQP-PAR.XQPP)./((XQP-PAR.XL).^2);
    z = (F2Q + (XQP-PAR.XL).*Iq + EDP);

    f_xy(3*ng+1:4*ng) = (-EDP + (XQ-XQP).*(Iq - c.*z) )./TQOP; % dEDP/dt
    f_xy(5*ng+1:6*ng) = -z./PAR.TQOPP; % dF2Q/dt
end

% Infinity bus
if PAR.infbus ~=0
    idx = PAR.infbus:ng:ns*ng;
    f_xy(idx) = 0;
end

% Loss of syncronism, stop updating model
if any( abs(DELTA(1)-DELTA(2:end)) > pi*1.1)
    f_xy = f_xy*0;
    g_xy = g_xy*0;
end

fg=([f_xy; g_xy]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Save control signals
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u = [PM; EF; PL; QL];

if isfield(PAR,'EXC')
    %     u = [u; EF_ref];
    u(PAR.EXC.u_idx) = EF_ref;
end
if isfield(PAR,'PSS')
    %     u = [u; u_PSS];
    u(PAR.PSS.u_idx) = u_PSS;
end
if isfield(PAR,'PMU')
    %     u = [u; FREQ];
    u(PAR.PMU.u_idx) = FREQ;
end
if isfield(PAR,'CONV')
    u(PAR.CONV.PQ_idx) = [Pinject(Cbus);Qinject(Cbus)];
    if isfield(PAR.CONV,'Storage')
        u(PAR.CONV.Storage.u_idx) = Echarge;
        u(PAR.CONV.Storage.Charge_ctrl.u_idx) = dP_charge;
    elseif isfield(PAR.CONV,'HVDC_wind')
        u(PAR.CONV.HVDC_wind.Gdc_idx) = y_HVDC_wind.Vdc;
        u(PAR.CONV.HVDC_wind.P1_idx) = y_HVDC_wind.P1;
        u(PAR.CONV.HVDC_wind.P2_idx) = y_HVDC_wind.P2;
        u(PAR.CONV.HVDC_wind.Pwind_idx) = y_HVDC_wind.Pwind;
        u(PAR.CONV.HVDC_wind.dP_K1_idx) = y_HVDC_wind.dP_K1;
        u(PAR.CONV.HVDC_wind.dP_K2_idx) = y_HVDC_wind.dP_K2;
        u(PAR.CONV.HVDC_wind.rotor_speed_idx) = y_HVDC_wind.rotor_speed;
        % u(PAR.CONV.Storage.u_idx) = Echarge;
        % u(PAR.CONV.Storage.Charge_ctrl.u_idx) = dP_charge;
    end
    if PAR.CONV.type == 2 || PAR.CONV.type == 3
       u(PAR.CONV.DELTA_idx) = y_CONV.DELTAC;
       u(PAR.CONV.OMEGA_idx) = y_CONV.OMEGAC;
    end
    u(PAR.CONV.Ed_idx) = y_CONV.Ed;
    u(PAR.CONV.Eq_idx) = y_CONV.Eq;
    u(PAR.CONV.Id_idx) = y_CONV.Id;
    u(PAR.CONV.Iq_idx) = y_CONV.Iq;
    u(PAR.CONV.Pref_idx) = y_CONV.Pref;
    u(PAR.CONV.Qref_idx) = y_CONV.Qref;
end



if isfield(PAR,'WIND')
    %     u = [u; dPwind; dPwind_ref; d_wind_rot_speed];
    u(PAR.WIND.u_idx) = dPwind;
    u(PAR.WIND.FFR.u_idx) = dPwind_ref;
    u(PAR.WIND.SPEED.u_idx) = d_wind_rot_speed;
end

if isfield(PAR,'PODP')
    %     u = [u; FREQ_PODP; P_PODP];
    u(PAR.PODP.PLL.u_idx) = FREQ_PODP;
    u(PAR.PODP.u_idx) = u_P_PODP;
    if isfield(PAR.PODP,'input_func_filter')
        u(PAR.PODP.input_func_filter.u_idx) = dy_PODP;
    end
end

if isfield(PAR,'PODQ')
    %     u = [u; Q_PODQ];
    u(PAR.PODQ.u_idx) = u_Q_PODQ;
    if PAR.PODQ.type == 3
        u(PAR.PODQ.K1.u_idx) = Q_1;
        u(PAR.PODQ.K2.u_idx) = Q_2;
        u(PAR.PODQ.Kaux.u_idx) = Q_aux;
    elseif PAR.PODQ.type == 4
        u(PAR.PODQ.K1.u_idx) = Q_1;
        u(PAR.PODQ.K2.u_idx) = Q_2;
        u(PAR.PODQ.COM.u_idx) = Q_COM;
        u(PAR.PODQ.Kaux.u_idx) = Q_aux;
    end
end

if isfield(PAR,'GOV')
    u(PAR.GOV.u_idx) = PM_ramp_error;
end

end

