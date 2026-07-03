%% main_CONV_study
% Code used to produce figures for:
%
% J. Björk, H. Ekestam, H. Hörnequist, and M. Javdani, 
% “Facilitating converter testability by partitioning synchronizing and frequency control,” 
% in 2025 IEEE PES ISGT Europe, 2025, pp. 1–5.


fig_save_opt = struct;
fig_save_opt.file_path = '';
fig_save_opt.file_format = 'pdf';
fig_save_opt.save_fig = false;
% fig_save_opt.file_format = 'jpg';
% saveas(fig,[file_path,'four_plus_two_bus.eps'],'epsc')

%% Draw network
opt_ISGT2025
[Gm, ~, ~, fig] = lin_fg_xy_manual(PAR,2,1); % Manual linearization method with option to draw network
camroll(90); pos = get(fig,'position'); set(fig,'position',[pos(1:2),8,4])

%% Bode diagram of synchronizing control loop
Bode_lin_sync_loop

%% Frequency sweep, frequency reference w_ref -> Injected power P_e
Freq_sweep_freq_loop

%% Sizing the freqeuncy response
Time_sizing_freq_ctrl

%% Simulation Example1
Time_test1

