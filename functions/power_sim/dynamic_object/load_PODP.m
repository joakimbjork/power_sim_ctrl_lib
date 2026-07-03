function [PAR,opt] = load_PODP(PAR,opt)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-22

% Load Dynamic Governor

%% Example
% K = s*25*1/(s/100+1); % Q/V [p.u./p.u.];
% K = K*PAR.Sb; % Q/V [Mvar/p.u.]
% c = zeros(PAR.nbus,1);
% c(6) = 1;
% opt.PODQ = diag(c)*K;

%%
Sb = PAR.Sb;
nbus = PAR.nbus;

if isfield(opt,'PODP')
    if isfield(opt, 'PODP_max_out')
        max_out = opt.PODP_max_out;
    else
        max_out = [];
    end
    
    if isfield(opt, 'PODP_ramp_rate')
        ramp_rate = opt.PODP_ramp_rate;
    else
        ramp_rate = [];
    end
    
    if isfield(opt, 'PODP_dead_band')
        dead_band = opt.PODP_dead_band;
    else
        dead_band = [];
    end

    if isfield(opt, 'PODP_ramp_rate_anti_drift')
        ramp_rate_anti_drift = opt.PODP_ramp_rate_anti_drift;
    else
        ramp_rate_anti_drift = [];
    end
    
    
    if ~isfield(opt, 'PODP_type')
        opt.PODP_type = 1;
    end
    
    
    
    [G, PAR] = ss_object(1, opt.PODP/Sb, PAR,  max_out/Sb, ramp_rate/Sb, dead_band, ramp_rate_anti_drift);
    PAR.PODP = G; % Main control block
    % All POD-P implementation use this simplified PLL by default
    m = size(opt.PODP,1);
    if isfield(opt, 'PODP_freq_filter')
        [G, PAR] = ss_object(1, opt.PODP_freq_filter, PAR);
        PAR.PODP.PLL = G;
    else
        s = tf('s');
        F = 1/(2*pi) * s*(100/(s+100))^1; % Angle [rad] -> Freq [Hz]
        F = eye(m)*F;
        [G, PAR] = ss_object(1, F, PAR);
        PAR.PODP.PLL = G;
    end
    

    if opt.PODP_type == 1 
        
        if opt.PODP_type == 1
            disp('Loading linear PODP: FREQ [Hz] -> P [MW]')
        end
    elseif opt.PODP_type == 2 % OBS this is to show the probelm with deadband/exponential combo. 
        if opt.PODP_type == 2
            warning('This control law is not recomended! It is likely to destablize the local mode once PODP is triggered. Loading PODP with exponential gain block: FREQ [Hz] -> P [MW]')
            PAR.PODP.type = 2;
        end  
        [G, PAR] = ss_object(1, opt.PODP1, PAR,  [], [], []);
        PAR.PODP.K1 = G; % Initial control block
        if isfield(opt, 'PODP_dead_band2')
        dead_band2 = opt.PODP_dead_band2;
        else
            dead_band2 = [];
        end
        [G, PAR] = ss_object(1, opt.PODP2, PAR,  [], [], dead_band2);
        PAR.PODP.K2 = G; % Second control block (after deadband)
        PAR.PODP.A = opt.PODP_exp_gain; % Exponential control block
        % Exponential control block sends input to main control block PAR.PODP
    end    
    
    if isfield(opt, 'PODP_CL') % Should test be perfromed in closed loop
        PAR.PODP.CL = opt.PODP_CL;
    else
        PAR.PODP.CL = 1; % Loop is closed by default
    end

    if isfield(opt,'PODP_input_func_filter')
        [G, PAR] = ss_object(1, opt.PODP_input_func_filter, PAR,  [], [], []);
        PAR.PODP.input_func_filter = G; % Initial control block        
    end
    
end