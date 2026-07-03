function [out1, out2] = HVDC_wind_object(create_object, in1, in2, in3, in4)
% Joakim Björk, joakim.bjork@svk.se, 2024-07-09

% ss_object(1, opt.GOV/(Sb*2*pi), PAR, max_out/Sb, ramp_rate/Sb)
if create_object
    obj = in1; % Given data object of HVDC_wind_object
    
    Cbus = obj.bus;
    m = length(Cbus);

    if exist('in2','VAR')
        PAR = in2;
    else
        PAR = struct();
        PAR.PC = zeros(m,1);
        PAR.PQ = zeros(m,1);
    end
    % X = obj.X;

    P0 = PAR.PC; % OBS HVDC_wind_object need to be created after CONV_object
    Q0 = PAR.QC;

    idx_nx = [];

    G = struct();

    G.type = obj.type;

    % HVDC link 
    [G_,PAR] = ss_object(2, obj.Gdc, PAR); % Link dynamik: Vdc = 1/(s*C) * (P1-P2)
    G.Gdc = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, obj.K1, PAR); % DC controller on wind turbine side
    G.K1 = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, obj.K2, PAR); % DC controller on grid converter side
    G.K2 = G_;
    idx_nx = [idx_nx, G_.idx_nx];
    
    % Linearized wind turbine (https://doi.org/10.1109/TPWRS.2021.3104905) 
    [G_,PAR] = ss_object(2, obj.WTpower, PAR); % P_WT = (s-z)/(s+p) * (P1ref-P1); 
    G.WTpower = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, obj.WTconv, PAR); % P1 = 1/(s*Te+1) * P_WT (+P0); 
    G.WTconv = G_;
    idx_nx = [idx_nx, G_.idx_nx];

    [G_,PAR] = ss_object(2, obj.WTspeed, PAR); % speed_WT = -1/(J*(s+p)) * (P1ref-P1); 
    G.WTspeed = G_;
    idx_nx = [idx_nx, G_.idx_nx];

   

    

    % [G_,PAR] = ss_object(1, obj.Id_ctrl, PAR);
    % G.Id_ctrl = G_;
    % idx_nx = [idx_nx, G_.idx_nx];
    %
    % [G_,PAR] = ss_object(1, obj.Iq_ctrl, PAR);
    % G.Iq_ctrl = G_;
    % idx_nx = [idx_nx, G_.idx_nx];

    G.idx_nx = idx_nx; % Indeces of converter controller;



    G.Vdc0 = 1*ones(m,1);
    G.MBASE = obj.MBASE;
    G.P0 = P0;
    G.Q0 = Q0;
    G.Cbus = Cbus;

    %% Index for saving outputs in simulation
    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.Gdc_idx = u_count + u_idx; % DC voltage
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.P1_idx = u_count + u_idx; % Power realized by turbine side converter
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.P2_idx = u_count + u_idx; % Power realized by grid side converter
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.Pwind_idx = u_count + u_idx; % Power requested by wind turbine
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.dP_K1_idx = u_count + u_idx; % Power requested by turbine side dc voltage controller
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.dP_K2_idx = u_count + u_idx; % Power requested by grid side dc voltage controller
    PAR.u_count = u_count + length(u_idx);

    u_count = PAR.u_count; % Number of output signals in the network model
    u_idx = 1:m;
    G.rotor_speed_idx = u_count + u_idx; % Wind turbine rotor speed deviation
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
    P2 = in3; % [y_CONV.Pe] % The grid side converter output
    Vdc_REF = in4; % [Vdc_REF] % DC voltage reference
    
    %% HVDC link
    % This requires Gdc to be strictly proper
    [~,~,C1,~] = ssdata(G.Gdc.sys); %
    x1 = x0(G.Gdc.idx_nx); % pick out states
    Vdc = C1*x1+G.Vdc0;
    [f_xy(G.K1.idx_nx), dP_K1] = ss_object(0, G.K1, x0, G.Vdc0-Vdc, zeros(m,1));
    [f_xy(G.K2.idx_nx), dP_K2] = ss_object(0, G.K2, x0, Vdc-G.Vdc0, zeros(m,1));
    
    % This requires WTconv to be strictly proper

    %% NEEDS TO BE UPDATED
    % [~,~,C1,~] = ssdata(G.WTconv.sys); %
    % x1 = x0(G.WTconv.idx_nx); % pick out states
    % P1 = C1*x1+G.P0;
    % 
    % [f_xy(G.Gdc.idx_nx), Vdc] = ss_object(0, G.Gdc, x0, P1-P2, G.Vdc0);
    % 
    % [f_xy(G.WTpower.idx_nx), Pwind] = ss_object(0, G.WTpower, x0, G.P0-P1, zeros(m,1));
    % 
    % [f_xy(G.WTconv.idx_nx), P1] = ss_object(0, G.WTconv, x0, Pwind+dP_K1, G.P0); % Output already calculated above
    % [f_xy(G.WTspeed.idx_nx), rotor_speed] = ss_object(0, G.WTspeed, x0, G.P0-P1, zeros(m,1));



    [f_xy(G.WTpower.idx_nx), Pwind] = ss_object(0, G.WTpower, x0, dP_K1, zeros(m,1));
    
    [f_xy(G.WTconv.idx_nx), P1] = ss_object(0, G.WTconv, x0, Pwind, G.P0); % Output already calculated above
    [f_xy(G.Gdc.idx_nx), Vdc] = ss_object(0, G.Gdc, x0, P1-P2, G.Vdc0);
    [f_xy(G.WTspeed.idx_nx), rotor_speed] = ss_object(0, G.WTspeed, x0, -dP_K1, zeros(m,1));
   

    %% Output
    out1 = f_xy(G.idx_nx); % Derivatives of converter dynamic objects
    out2 = struct();
    out2.Vdc = Vdc;
    out2.P1 = P1;
    out2.P2 = P2;
    out2.Pwind = Pwind;
    out2.dP_K1 = dP_K1;
    out2.dP_K2 = dP_K2;
    out2.rotor_speed = rotor_speed;
end

