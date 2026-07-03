function [PAR,opt] = load_HVDC_wind(PAR,opt)
% Joakim Björk, joakim.bjork@svk.se, 2026-03-03

% HVDC wind object.

%% Example
% Data based on NREL 5MW bencmark turbine
% P_nom = 5; % MW
% J = 44470000; %kg m^2
% v = 10; % m/s
% rot_mpp = 1.19; % rad/s
% P_mpp = 3.5; % MW
% z = 5.8*v*1e-3;
% TE = 0.02; % Time constant converter
% Gwind = (s-z)/(s+z)/(s*TE+1); % Pref [MW] -> Pe [MW]
% Gwind_speed = -1e6/(rot_mpp*J*(s+z))/(s*TE+1); % Pref [MW] -> rot [rad/s]
%
% % Number of 5 MW turbines participating in POD-P
% num_turbines = 10;
%
% % POD-P controller
% Tw = 10;
% k = 1000; % MW/Hz
% Kpod = k*(s*Tw)/(s*Tw+1); % Freq [Hz] -> Pref [MW]
% c = zeros(PAR.nbus,1);
% c(6) = 1;
% c = c/sum(c);
% opt.WIND = diag(c)*Gwind*Kpod; % Freq [Hz] -> Pref [MW]
% opt.WIND_speed = diag(c)*Gwind_speed*Kpod/num_turbines; % Freq [Hz] -> wind rot [rad/s]

%%
Sb = PAR.Sb;

if isfield(opt.CONV,'HVDC_wind')
    obj = opt.CONV.HVDC_wind;
    Cbus = obj.bus;
    m = length(Cbus);
    s=tf('s');

    %% Base parameters
    if ~isfield(obj, 'MBASE') % Size of inverter in pu power base. Ex: 100 MW inverter -> MBASE = 100/Sb;
        obj.MBASE = ones(m,1);
    end
    if ~isfield(obj, 'type')
        obj.type = 1;
    end

    %% DC control
    C = diag(obj.MBASE)*1e-2;
    % By default, the HVDC link controller is implemented as a PID control
    % where the PI part is implemented in the grid side (K2) and D part in
    % the turbine side (K1);
    K1 = tf(zeros(m));
    K2 = tf(zeros(m));
    for i = 1:m
        Ci = C(i,i);
        wc1 = 0.1; % Desired crossover frequecy of turbine side control. The turbine side will control above this. So wc_turbine > RHP zeros of the wind turbine
        wc2 = 1000; % Desired crossover frequency of grid side dc control
        wc3 = wc1*10; % Bandwidth of the derivative controller
        kp = wc2*Ci;
        ki = 0.1*wc1*kp; % This should not interfere with the derivative control. Therefore we tune it to be one decade below
        kD = sqrt(kp^2-wc1^2*Ci^2)/wc1; % Alt2
        K1(i,i) = s*kD*wc3/(s+wc3);
        K2(i,i) = kp+ki/s;
    end
    % HVDC link
    if ~isfield(obj, 'Gdc') % DC-link dynamics
        K = 1/s*inv(C);
        obj.Gdc = minreal(eye(m)*K);
    end

    if ~isfield(obj, 'K1') % DC controller on wind turbine side
        obj.K1 = K1; % Use K1 calculated above
    end

    if ~isfield(obj, 'K2') % DC controller on grid converter side
        obj.K2 = K2; % Use K1 calculated above
    end


    % Linearized wind turbine (https://doi.org/10.1109/TPWRS.2021.3104905)
    if ~isfield(obj, 'WTpower')
        % v = 10; % m/s
        % rot_mpp = 1.19; % rad/s
        % P_mpp = 3.5; % MW
        % z = 5.8*v*1e-3;
        v = 10; % m/s
        v = 8; % m/s
        z = 5.8*v*1e-3; % = 0.058
        p = z;
        K = (s-z)/(s+z); % P_WT = (s-z)/(s+p) * (P1ref-P1);
        K = (s-0.01)/(s+0.01);
        % K = tf(1);
        obj.WTpower = minreal(eye(m)*K);
    end

    if ~isfield(obj, 'WTconv')
        TE = 1e-4; % Converter time constant
        K = 1/(s*TE+1);  % P1 = 1/(s*TE+1) * P_WT (+P0);
        K = tf(1);
        obj.WTconv = minreal(eye(m)*K);
    end

    if ~isfield(obj, 'WTspeed')
        v = 10; % m/s
        z = 5.8*v*1e-3;
        Sb_nom = 5; % Trubine based on 5 MW baseline model
        J = 44470000*obj.MBASE*PAR.Sb/Sb_nom; %kg m^2
        rot_mpp = 1.19; % rad/s
        rot_mpp = 0.95;
        Minv = 1./(rot_mpp.*J);
        Minv = diag(Minv);
        K = eye(m)*1/(s+z)*Minv*PAR.Sb*1e6;  % speed_WT = -1e6/(rot_mpp*J*(s+z)) * (P1ref-P1);  Pref [MW] -> rot [rad/s]
        obj.WTspeed = minreal(K);
    end

    [G, PAR] = HVDC_wind_object(1, obj, PAR);

    PAR.CONV.HVDC_wind = G;


end