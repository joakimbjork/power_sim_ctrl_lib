function [PAR,opt] = load_PMU(PAR,opt)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-22

% Load phasor measurement unit (freqeuncy measurement)

%% Example
% s= tf('s');
% PMU = 1/(2*pi) * s*(100/(s+100))^2; % Angle [rad] -> Freq [Hz]
% opt.PMU = eye(PAR.nbus)*PMU; 

%%
nbus = PAR.nbus;
% Sb = PAR.Sb;

if ~isfield(opt, 'PMU_max_out')
    opt.PMU_max_out = ones(nbus,1).*[-1e6,1e6];
end

if isfield(opt,'PMU')  
    [G, PAR] = ss_object(1, opt.PMU, PAR, opt.PMU_max_out);
    PAR.PMU = G;
end