function [PAR,opt] = load_GOV(PAR,opt)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-22
% Load Dynamic Governor
%% Example
% k = 1400/0.4; % Static gain [MW/Hz];
% s = tf('s');
% Fgov = k*(1+7*s)/((1+2*s)*(1+14*s)); % Dynamic governor [MW/Hz];
%%% Fgov = Fgov*(2-s)/(2+s); NMP dynmaic gov (e.g. hydro power)
% c = zeros(PAR.ng,1);
% c(1:3) = [0.6,0.3,0.1];
% c = c/sum(c);
% opt.GOV = c*Fgov;

%%
Sb = PAR.Sb;
ng = PAR.ng;

if isfield(opt,'GOV')
    if isempty(opt.GOV)
        disp('Loading default governors with gain Mbase/20 [MW/Hz]')
        s = tf('s');
        % Fgov = (1+7*s)/((1+2*s)*(1+14*s));%*(2-s)/(2+s); % Dynamic regulator/governor [MW/Hz];
        Fgov = (1+7*s)/((1+3*s)*(1+14*s));%*(2-s)/(2+s); % Dynamic regulator/governor [MW/Hz];
        c = 5*diag(PAR.M)*Sb;
        opt.GOV = Fgov*c;
    end

    if ~isfield(opt, 'GOV_type')
        opt.GOV_type = 1;
    end
    if isfield(opt, 'GOV_max_out')
        max_out = opt.GOV_max_out;
    else
        max_out = [];%PAR.MBASE.*[0,1]*Sb;
    end
    
    if isfield(opt, 'GOV_ramp_rate')
        ramp_rate = opt.GOV_ramp_rate;
    elseif opt.GOV_type == 1
        ramp_rate = [];
    else
        disp('Default ramp rate limit');
        ramp_rate = 0.1*PAR.MBASE.*[-1,1]*Sb;
    end
    
    
    if isempty(ramp_rate)
        disp('Loading dynamic governors: FREQ [Hz] -> P [MW]')
        [G, PAR] = ss_object(1, opt.GOV/(Sb*2*pi), PAR, max_out/Sb, ramp_rate/Sb);
    else
        disp('Loading dynamic governors with ramp rate limiter: FREQ [Hz] -> P [MW]')
        if isfield(opt, 'GOV_antirampdrift') && ~isempty(opt.GOV_antirampdrift)
            [G, PAR] = ss_object(1, opt.GOV/(Sb*2*pi), PAR, max_out/Sb, ramp_rate/Sb, [], opt.GOV_antirampdrift); 
        else
            [G, PAR] = ss_object(1, opt.GOV/(Sb*2*pi), PAR, max_out/Sb, ramp_rate/Sb);
        end
    end    
    PAR.GOV = G;

    if isfield(opt, 'GOV_CL') % Should test be perfromed in closed loop
        PAR.GOV.CL = opt.GOV_CL;
    else
        PAR.GOV.CL = 1; % Loop is closed by default
    end
end