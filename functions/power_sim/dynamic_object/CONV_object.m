function [out1, out2] = CONV_object(create_object, in1, in2, in3, in4)
% Joakim Björk, joakim.bjork@svk.se, 2024-07-09

% ss_object(1, opt.GOV/(Sb*2*pi), PAR, max_out/Sb, ramp_rate/Sb)
if create_object
    obj = in1; % Given data object of converter
    if exist('in2','VAR')
        PAR = in2;
    else
        PAR = struct();
    end
    Cbus = obj.bus;
    m = length(Cbus);
    X = obj.X;

    PC = PAR.P0(Cbus);
    QC = PAR.Q0(Cbus);
    PAR.PC = PC;
    PAR.QC = QC;

    PAR.P0(Cbus) = PC - PAR.P0(Cbus);
    PAR.Q0(Cbus) = QC - PAR.Q0(Cbus);

    % STEP 1: Load-flow gives the voltages at converter buses
    U=PAR.U0(Cbus);
    ANGC=PAR.ANG0(Cbus);
    UC = U.*exp(1j*ANGC); % Complex voltage vector at converter buses

    % STEP 2: Calculate phase angle
    % Together with the injected power PC + j QC, we get the injected current
    IC=conj((PC+j*QC)./(UC));
    EC =1j*X.*IC + UC; % Internal voltage of the converter
    if obj.type == 3
        DELTAC=angle(EC);
        % DELTAC=ANGC;
    else % Use a grid following convention where netwrok node is used as reference
        DELTAC=ANGC;
    end
    % STEP 3: dq-values of converter current and voltages
    Idq = IC.*exp(1j*(-DELTAC));
    Id0 = real(Idq);
    Iq0 = imag(Idq);
    Udq = UC.*exp(1j*(-DELTAC));
    Ud = real(Udq);
    Uq = imag(Udq);
    Edq = EC.*exp(1j*(-DELTAC));
    Ed0 = real(Edq);
    Eq0 = imag(Edq);

    % PC1 = Ud.*Id0 + Uq.*Iq0;
    % PC2 = Ed0.*Id0 + Eq0.*Iq0 % No losses -> PC1 = PC2
    % QC = Id0.*Uq - Iq0.*Ud; % Reactive power injected at the network node

    % EC = Ed + j*Eq = j*X*(Id+j*Iq) + Ud + j*Uq = -X.*Iq + Ud + j*( X.*Id + Uq)
    % Ed = Ud-X.*Iq; -> Ud = Ed0 + X.*Iq0;
    % Eq = X.*Id + Uq; -> Uq = Eq0 - X.*Id0;
    % QC = Id0.*(Eq0 - X.*Id0) - Iq0.*(Ed0 + X.*Iq0); % Reactive power injected at the network node

    idx_nx = [];

    G = struct();

    G.type = obj.type;

    if G.type == 2 || G.type == 3
        [G_,PAR] = ss_object(2, obj.phase_ctrl, PAR); % OMEGAC [rad/s] -> ANG[rad]
        G.phase_ctrl = G_; %
        idx_nx = [idx_nx, G_.idx_nx];
        [G_,PAR] = ss_object(2, obj.sync_ctrl, PAR); % dUq (type 2) or dP (type 3) -> OMEGAC [rad/s]
        G.sync_ctrl = G_; %
        idx_nx = [idx_nx, G_.idx_nx];
    end

    [G_,PAR] = ss_object(2, obj.P_meas, PAR);
    G.P_meas = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, obj.Q_meas, PAR);
    G.Q_meas = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, obj.freq_meas, PAR);
    G.freq_meas = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    % [G_,PAR] = ss_object(2, obj.freq_filter, PAR);
    % G.freq_filter = G_;
    % idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, ss(eye(m)), PAR, obj.P_max_out/PAR.Sb);
    G.P_lim = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, ss(eye(m)), PAR, obj.Q_max_out/PAR.Sb);
    G.Q_lim = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, obj.Q_ctrl, PAR);
    G.Q_ctrl = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    if obj.type == 3 % Grid forming already has power control
        % [G_,PAR] = ss_object(2, obj.Ed_ctrl, PAR);
        % G.Ed_ctrl = G_;
        % idx_nx = [idx_nx, G_.idx_nx];
    else
        [G_,PAR] = ss_object(2, obj.P_ctrl, PAR);
        G.P_ctrl = G_;
        idx_nx = [idx_nx, G_.idx_nx];
    end
    [G_,PAR] = ss_object(2, obj.Id_ctrl, PAR);
    G.Id_ctrl = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, obj.Iq_ctrl, PAR);
    G.Iq_ctrl = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    

    % [G_,PAR] = ss_object(1, obj.Id_ctrl, PAR);
    % G.Id_ctrl = G_;
    % idx_nx = [idx_nx, G_.idx_nx];
    %
    % [G_,PAR] = ss_object(1, obj.Iq_ctrl, PAR);
    % G.Iq_ctrl = G_;
    % idx_nx = [idx_nx, G_.idx_nx];

    G.idx_nx = idx_nx; % Indeces of converter controller;



    G.X = X;
    G.MBASE = obj.MBASE;
    if obj.type == 3
        G.ki_sync = obj.ki_sync;
        G.kp_sync = obj.kp_sync;
    end
    G.Imax = obj.Imax;
    G.Imax_inner = obj.Imax_inner;
    G.Id0 = Id0;
    G.Iq0 = Iq0;
    G.Ed0 = Ed0;
    G.Eq0 = Eq0;
    G.Uq0 = Uq;
    G.Ud0 = Ud;
    G.ANGC0 = ANGC;
    G.DELTAC0 = DELTAC;
    G.P0 = PC;
    G.Q0 = QC;
    G.Cbus = Cbus;

    %% Index for saving outputs in simulation
    % Index for saving active and reactive power output
    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m*2;
    G.PQ_idx = u_count + u_idx;
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.DELTA_idx = u_count + u_idx;
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.OMEGA_idx = u_count + u_idx;
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.Ed_idx = u_count + u_idx;
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.Eq_idx = u_count + u_idx;
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.Id_idx = u_count + u_idx;
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.Iq_idx = u_count + u_idx;
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.Pref_idx = u_count + u_idx;
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.Qref_idx = u_count + u_idx;
    PAR.u_count = u_count + length(u_idx);

    out1 = G;
    out2 = PAR;
else
    %% Simulate existing ss_object
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % u = u0 + du = u0 + G*y

    G = in1; m = length(G.Cbus);
    x0 = in2; f_xy = 0*x0;
    % y = in3; % [UC; ANGC]
    % u0 = in4; % [P_REF; Q_REF]
    U = in3(1:m); % Voltage amplitude at network node
    ANGC = in3(m+(1:m)); % Voltage phase angle at network node
    UC = U.*exp(1j*ANGC);

    if G.type == 1 % --> Perfect alignment
        DELTAC = ANGC;
    else % Grid forming/following gets phase angle from integrator block "G.phase_ctrl"
        [~, DELTAC] = ss_object(0, G.phase_ctrl, x0, 0*G.DELTAC0, G.DELTAC0); % Output is DELTAC
    end

    Pref = in4(1:m);
    Qref = in4(m+(1:m));

    % Qref = min( abs(Qref) , G.MBASE/3).*sign(Qref);

    Udq = UC.*exp(1j*(-DELTAC));
    Ud = real(Udq);
    Uq = imag(Udq);

    % This requires Idq control blocks to be strictly proper
    [~,~,C1,~] = ssdata(G.Id_ctrl.sys); %
    x1 = x0(G.Id_ctrl.idx_nx); % pick out states
    Id = C1*x1+G.Id0;
    [~,~,C1,~] = ssdata(G.Iq_ctrl.sys); %
    x1 = x0(G.Iq_ctrl.idx_nx); % pick out states
    Iq = C1*x1+G.Iq0;

    % Power injected at network node
    Pe = Iq.*Uq + Id.*Ud;
    Qe = Id.*Uq - Iq.*Ud;

    Idq = Id+ 1j*Iq;
    % IC = Idq.*exp(1j*(DELTAC));
    % EC =1j*G.X.*IC + UC; % Internal voltage of the converter
    % Edq = EC.*exp(1j*(-DELTAC));
    Edq = 1j*G.X.*Idq + Udq;
    Ed = real(Edq);
    Eq = imag(Edq);

    % EC =1j*G.X.*IC + UC; % Internal voltage of the converter
    % IC = -1j*(EC - UC)./G.X = -1j*(Ed+j*Eq - Ud - j*Uq)/X

    % [f_xy(G.P_lim.idx_nx), P_ref] = ss_object(0, G.P_lim, x0, P_ref-G.P0, G.P0); %
    % [f_xy(G.Q_lim.idx_nx), Q_ref] = ss_object(0, G.Q_lim, x0, Q_ref-G.Q0, G.Q0);
    if G.type == 3
        %% Syncronizing control (Grid forming)
         
        % Current limitation
        Idq_ref = conj((Pref+1j*Qref)./Udq);
        Idq_lim = min( abs(Idq_ref) , G.Imax).*exp(1j*angle(Idq_ref));
        Id_lim = real(Idq_lim);
        Iq_lim = imag(Idq_lim);
        P_lim = Iq_lim.*Uq + Id_lim.*Ud;
        Q_lim = Id_lim.*Uq - Iq_lim.*Ud;

        % Grid forming control
        [f_xy(G.sync_ctrl.idx_nx), OMEGAC] = ss_object(0, G.sync_ctrl, x0, (P_lim-Pe), zeros(m,1));
        [f_xy(G.phase_ctrl.idx_nx), DELTAC] = ss_object(0, G.phase_ctrl, x0, OMEGAC, G.DELTAC0); %

        [f_xy(G.Q_ctrl.idx_nx), Ed_ref] = ss_object(0, G.Q_ctrl, x0, (Q_lim-Qe), G.Ed0); % Qe is proportional to "Ed" OBSOLETE?
        %% Outer control
        % Current that would be realized by Ed_ref
        Id_ref = -Uq./G.X;% (Eq_Ref - Uq)./G.X, where Eq_ref = 0
        Iq_ref = (Ud-Ed_ref)./G.X;
        
        % Current limitation (OBS sligthly larger amplitude than inital limiter)
        Idq_ref = Id_ref+1j*Iq_ref;
        % Idq_ref = (Ed_ref - Udq)./(1j*G.X);
        Idq_lim = min( abs(Idq_ref) , G.Imax_inner).*exp(1j*angle(Idq_ref));
        Id_lim = real(Idq_lim);
        Iq_lim = imag(Idq_lim);
        
        % Not modelling the internal voltage controller since we then also
        % have to model a current filter (capacitor) to manage over current. 
        % Idq_lim = (Ed_lim - Udq)./(1j*G.X);
        % (Ed_lim - Udq) = Idq_lim.*1j*G.X;
        % Ed_lim = Idq_lim.*1j*G.X + Udq;
        % [f_xy(G.Ed_ctrl.idx_nx), Ed] = ss_object(0, G.Ed_ctrl, x0, Ed_lim-G.Ed0, G.Ed0);
    elseif G.type <= 2
        %% Syncronizing control (Grid following = 2)
        if G.type == 2 % Grid following PLL control
            [f_xy(G.sync_ctrl.idx_nx), OMEGAC] = ss_object(0, G.sync_ctrl, x0, Uq-G.Uq0, zeros(m,1));
            [f_xy(G.phase_ctrl.idx_nx), DELTAC] = ss_object(0, G.phase_ctrl, x0, OMEGAC, G.DELTAC0); % Output is DELTAC
        else
            OMEGAC = DELTAC*0;
        end
      
        %% Outer control
        % Current limitation
        Idq_ref = conj((Pref+1j*Qref)./Udq);
        Idq_lim = min( abs(Idq_ref) , G.Imax).*exp(1j*angle(Idq_ref));
        Id_lim = real(Idq_lim);
        Iq_lim = imag(Idq_lim);
        P_lim = Iq_lim.*Uq + Id_lim.*Ud;
        Q_lim = Id_lim.*Uq - Iq_lim.*Ud;

        [f_xy(G.P_ctrl.idx_nx), Id_lim] = ss_object(0, G.P_ctrl, x0, P_lim-Pe, G.Id0);
        [f_xy(G.Q_ctrl.idx_nx), Iq_lim] = ss_object(0, G.Q_ctrl, x0, -(Q_lim-Qe), G.Iq0); % Qe is proportional to "- Iq"
    else
        OMEGAC = DELTAC*0;
        % Current limitation
        Idq_ref = conj((Pref+1j*Qref)./Udq);
        % Idq_lim = min( abs(Idq_ref) , G.Imax).*exp(1j*angle(Idq_ref));
        Idq_lim = Idq_ref;
        Id_lim = real(Idq_lim);
        Iq_lim = imag(Idq_lim);
    end
    %% Inner control
    [f_xy(G.Id_ctrl.idx_nx), Id] = ss_object(0, G.Id_ctrl, x0, Id_lim-G.Id0, G.Id0);
    [f_xy(G.Iq_ctrl.idx_nx), Iq] = ss_object(0, G.Iq_ctrl, x0, Iq_lim-G.Iq0, G.Iq0);

    [f_xy(G.P_meas.idx_nx), P_meas] = ss_object(0, G.P_meas, x0, Pe-G.P0, G.P0); % Output not used here, but updates dynamics in measurement block for external use
    [f_xy(G.Q_meas.idx_nx), Q_meas] = ss_object(0, G.Q_meas, x0, Qe-G.Q0, G.Q0); % Output not used here, but updates dynamics in measurement block for external use
    [f_xy(G.freq_meas.idx_nx), freq_meas] = ss_object(0, G.freq_meas, x0, OMEGAC, zeros(m,1)); % Output not used here, but updates dynamics in measurement block for external use


    %% Output
    out1 = f_xy(G.idx_nx); % Derivatives of converter dynamic objects
    out2 = struct();
    out2.Pe = Pe;
    out2.Pref = Pref;
    out2.Qref = Qref;
    out2.Qe = Qe;
    out2.DELTAC = DELTAC;
    out2.OMEGAC = OMEGAC;
    out2.Ed = Ed;
    out2.Eq = Eq;
    out2.Id = Id;
    out2.Iq = Iq;
end

