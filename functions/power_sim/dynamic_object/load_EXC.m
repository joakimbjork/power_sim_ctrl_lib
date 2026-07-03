function [PAR,opt] = load_EXC(PAR,opt)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-22

% Load Dynamic Excitation

%% Example
% s= tf('s');
%%% Static excitation system
% TE = 0.05; 
% Gexc = 1/(s*TE+1);

%%% Automatic voltage controller (AVR)
% wA = pi/10;
% wB = wA/4;
% Kavr = 40*(s/wA+1)/(s/wB+1); % field voltage/measured voltage [p.u./p.u.]
% EXC = Gexc*Kavr;

%opt.EXC_max_out = ones(PAR.ng,1).*[0,5];
%opt.EXC = eye(PAR.ng)*EXC; % 

%%

ng = PAR.ng;
if isfield(opt,'EXC')
    
    if ~isfield(opt, 'EXC_max_out')
        opt.EXC_max_out = ones(ng,1).*[0,3];
    end

%% Exciter   
    if ~isfield(opt, 'EXC_ramp_lim')
        [G, PAR] = ss_object(1, opt.EXC, PAR,  opt.EXC_max_out);
    else
        if size(opt.EXC_ramp_lim,1) == 1
            opt.EXC_ramp_lim = opt.EXC_ramp_lim(1,1)*ones(ng,1);
        end
        [G, PAR] = ss_object(1, opt.EXC, PAR, opt.EXC_max_out, opt.EXC_ramp_lim);
    end   
    disp('Loading dynamic exciter with AVR: VOLT [p.u.] -> VOLT [p.u.]')     
    PAR.EXC = G;
    PAR.EXC.CL = 1;
    if isfield(opt, 'EXC_type')
        disp('EXC_type 1/2 is replased by closed-loop/open-loop specification option')
        if opt.EXC_type == 1
            PAR.EXC.CL = 1;
        elseif PAR.EXC.CL == 2
            PAR.EXC.CL = 0;
        end
    end
    if isfield(opt, 'EXC_CL') % Should test be performed in closed loop
        assert(~isfield(opt, 'EXC_type'), 'EXC_type option is obsolete')
        PAR.EXC.CL = opt.PSS_CL;
    end
        

%% AVR
    % Standard is to have AVR togehter with EXC
    % if isfield(opt, 'AVR')
    %     if ~isfield(opt, 'AVR_max_out')
    %        opt.AVR_max_out = ones(ng,1).*[-1,1]*100;
    %     end
    %     disp('Loading AVR: VOLT [p.u.] -> VOLT [p.u.]')
    %     [G, PAR] = ss_object(1, opt.AVR, PAR,  opt.AVR_max_out);
    %     PAR.AVR = G;
    % end

%% PSS
    if isfield(opt, 'PSS')
        if ~isfield(opt, 'PSS_max_out')
           opt.PSS_max_out = ones(ng,1).*[-1,1]*100;
        end
        disp('Loading PSS: FREQ [Hz] -> VOLT [p.u.]')
        [G, PAR] = ss_object(1, opt.PSS/(2*pi), PAR,  opt.PSS_max_out); 
        PAR.PSS = G;

        if isfield(opt, 'PSS_type') % 
            PAR.PSS.type = opt.PSS_type;
        else
            PAR.PSS.type = 1; % 
        end

        if isfield(opt, 'PSS_CL') % Should test be perfromed in closed loop
            PAR.PSS.CL = opt.PSS_CL;
        else
            PAR.PSS.CL = 1; % Loop is closed by default
        end

    end
end