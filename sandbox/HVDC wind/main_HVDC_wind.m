%% main_HVDC_wind
% 
% Simulation example combining partitioned GFM control architecture [1] on 
% a HVDC interconnected variable speed wind turbine [2] 
%
% [1] J. Björk, H. Ekestam, H. Hörnequist, and M. Javdani, 
%     “Facilitating converter testability by partitioning synchronizing and frequency control,” 
%     in 2025 IEEE PES ISGT Europe, 2025, pp. 1–5.
%
% [2] J. Björk, D. V. Pombo, and K. H. Johansson, 
%     “Variable-speed wind turbine control designed for coordinated fast frequency reserves,” 
%     IEEE Trans. Power Syst., vol. 37, no. 2, pp. 1471– 1481, Mar. 2022.

fig_save_opt = struct;
fig_save_opt.file_path = '';
fig_save_opt.file_format = 'pdf';
fig_save_opt.save_fig = false;

%% Draw network
opt_HVDC_wind
[Gm, ~, ~, fig] = lin_fg_xy_manual(PAR,2,1); % Rita figur (manuell linjärisering)
camroll(90); pos = get(fig,'position'); set(fig,'position',[pos(1:2),8,4])

%%
time_test1_HVDC_wind

%%
freq_sweep1_HVDC_wind

%% DC control tuning
s = tf('s');
MBASE = 20;
Ci = MBASE*1e-3;
aD = 0.1; % Desired crossover frequency of turbine side control. The turbine side will control above this. So aD > RHP zeros of the wind turbine
ap = 1000; % Desired crossover frequency of grid side dc control
aD2 = aD*10; % Bandwidth of the derivative controller
kp = ap*Ci;
ki = 0.1*aD*kp; % This should not interfere with the derivative control. Therefore we tune it to be one decade below
kD = sqrt(kp^2-aD^2*Ci^2)/aD; % 
K1 = s*kD*aD2/(s+aD2);
K2 = kp+ki/s;
Gdc = 1/(s*Ci);
Gdc1 = Gdc/(1+Gdc*K2);

[fig3,~,co] = figureLatex(3.5,3.5/sqrt(2));

bodeopt = struct();
% bodeopt.wrap = true;
% bodeopt.adjust = false;
% bodeopt.FreqUnits = 'Hz';
bodeopt.plot_phase = false;
bodeopt.omega_lim = [1e-4,1e6] ;

bodeopt = bodeLatex(Gdc*(K2),bodeopt); hold all
bodeopt = bodeLatex(Gdc*(K2+K1),bodeopt);
bodeopt = bodeLatex(Gdc1*(K1),bodeopt);
for i = 1:1
    nexttile(i)
    xline(0.1*aD); hold all
    xline(aD); hold all
    xline(aD2); hold all
    xline(ap); hold all
    grid on
end

ylims = ylim;
ydiff = log10(abs(diff(ylims)));
y_loc = ylims(1);
h = text(ap, y_loc,...
    '$\omega_{p}$\hspace{1pt}','HorizontalAlignment','right',...
    'VerticalAlignment','bottom');
h = text(0.1*aD, y_loc,...
    '$0.1\omega_{d}$\hspace{1pt}','HorizontalAlignment','right',...
    'VerticalAlignment','bottom');
h = text(aD, y_loc,...
    '\hspace{2pt}$\omega_{d}$','HorizontalAlignment','left',...
    'VerticalAlignment','bottom');
h = text(aD2, y_loc,...
    '\hspace{2pt}$\omega_{d2}$','HorizontalAlignment','left',...
    'VerticalAlignment','bottom');

l = legendLatex(fig3,{'$(sC)^{-1}K_2$','$(sC)^{-1}(K_1+K_2)$','$(sC+K_2)^{-1}K_1$'}); 
set(l,'location','northeast','Orientation','vertical')


