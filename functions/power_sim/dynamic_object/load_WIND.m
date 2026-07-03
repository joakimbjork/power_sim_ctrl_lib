function [PAR,opt] = load_WIND(PAR,opt)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-22

% Load Dynamic Governor

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
nbus = PAR.nbus;
Sb = PAR.Sb;

if isfield(opt,'WIND')  
    assert(isfield(opt,'PMU'),'Wind turbine needs a PMU since it uses frequency measurement')  
    if ~isfield(opt, 'WIND_type')
        opt.WIND_type = 1;
    end
    if ~isfield(opt, 'WIND_max_out')
        opt.WIND_max_out = ones(nbus,1).*[-1e6,1e6];
    end
    if ~isfield(opt, 'WIND_FFR_max_out')
        opt.WIND_FFR_max_out = ones(nbus,1).*[-1e6,1e6];
    end
    
    if opt.WIND_type == 1
        disp('Loading linear wind turbine model')
        [G1, PAR] = ss_object(1, opt.WIND, PAR,  opt.WIND_max_out);
        [G2, PAR] = ss_object(1, opt.WIND_speed*Sb, PAR,  ones(nbus,1).*[-1e6,1e6]);
        [K, PAR] = ss_object(1, opt.WIND_FFR/(Sb*2*pi), PAR,  opt.WIND_FFR_max_out);       
        
        
        PAR.WIND = G1;       
        PAR.WIND.FFR = K;
        PAR.WIND.SPEED = G2;
    end   
end