%% main_CIGRE2024
%
% Code used to produce figures for:
%
% J. Björk, H. Ekestam, H. Hagmar, and M. Oluić, 
% “Simultaneous voltage and power oscillation damping control: Towards robust and scalable grid requirements and control solutions,” 
% in CIGRE 2024 Paris Session (C4).
% URL: https://www.e-cigre.org/publications/detail/c4-10495-2024-simultaneous-voltage-and-power-oscillation-damping-control-towards-robust-and-scalable-grid-requirements-and-control-solutions.html

fig_save_opt = struct;
fig_save_opt.file_path = '';
fig_save_opt.file_format = 'pdf';
fig_save_opt.save_fig = false;
figwidth = 4;
figheight = 3.5/sqrt(2);
%% Draw network
opt_CIGRE2024;
[Gm, ~, ~, fig] = lin_fg_xy_manual(PAR,2,1); % Manual linearization method with option to draw network
camroll(90); pos = get(fig,'position'); set(fig,'position',[pos(1:2),8,4])

%% Input-output selection for POD
Bode_in_out_pairing

%% Pure POD-Q control
Root_locus_gain_tuning % Tuning POD-Q controller using root locus

Bode_open_loop1

%% Voltage control 
Bode_open_loop2

Time_sim1 % Simulate voltage reference step using pure voltage control

%% Simultaneous POD-Q and voltage control
Bode_open_loop3

Time_sim2 % Simulate voltage reference step and/or load change event (change simulation by modifying "sim_i" parameter in Time_sim2.m)

%% POD-Q controller with deadband activated voltage control
Describing_function % Describing function Nyquist plot and Bode diagram

Time_sim3 % Simulate load change event