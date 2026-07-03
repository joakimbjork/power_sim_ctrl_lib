%% Default input 
opt_ISGT2025
% opt.CONV.MBASE = 10*ones(m,1); % Lower the rating to get more visible difference between ESCP and SCP
opt.CONV.Storage = ss(0*eye(m));
opt.PODP = opt.PODP*0 ;
% opt.PODQ = opt.PODQ*0 ;
opt.PC0 = zeros(m,1);
[PAR, opt] = data_Nordic5(opt); % Modell 

idx = 6; % <---- This bus is in Norway close to bus 1
nc = length(PAR.CONV.Cbus);

PAR.PODP.CL = ones(PAR.nbus,1);
PAR.PODP.CL(cbus(idx)) = 0; % <------------- Open POD-P control loop for the test

PAR.input = 'Pref_CONV';
PAR.output = 'PQ_CONV';
G = lin_fg_xy(PAR.xy0,PAR); 
Gp = G(idx,idx); % Power [pu] --> Power [pu] (same base)

T = Gp;
S = 1-T;
Lp = (inv(S)-1);
[Gm,Pm,Wcg,Wcp] = margin(Lp); % Warns about unstable. This is just a numerical issue

SCP = opt.data.SCP(PAR.CONV.Cbus(idx));
ESCP = opt.data.ESCP(idx);
s = tf('s');
Lp_est = ESCP/s;
Lp_est = Lp_est*PAR.CONV.sync_ctrl.sys(idx,idx);



% Closed loop system
[fig,name,co] = figureLatex(3.5,3.5/sqrt(2));
bodeopt = struct();
bodeopt.wrap = true;
% bodeopt.FreqUnits = 'Hz';
bodeopt.omega_lim = [1e-2,1e4] ;

name{1} = '$T$';
bodeopt.color = [1,1,1]*0; 
bodeopt = bodeLatex(Gp,bodeopt); hold all

name{2} = '$L$';
bodeopt.color =  co(1,:); 
bodeopt = bodeLatex(Lp,bodeopt); hold all

name{3} = '${L}_\mathrm{SCR}$';
bodeopt.linestyle = '--'; bodeopt.color = co(3,:); 
bodeopt = bodeLatex(Lp_est,bodeopt); hold all

nexttile(1)
ylim([1e-6,1e6])

w1 = 0.25*2*pi;
w2 = 1.0*2*pi;

ki = PAR.CONV.ki_sync(idx);
wc_est = sqrt(ESCP*ki);
wp_est = ESCP*PAR.CONV.kp_sync(idx);
for i = 1:2
    nexttile(i)
    xline(w1); hold all
    xline(w2); hold all
    xline(wp_est); hold all
    grid on
end
nexttile(1);
set(gca, 'ytick', [10^-4,1,10^4]);

nexttile(1);
ylims = ylim;
ydiff = log10(abs(diff(ylims)));
y_loc = ylims(1);
h = text(w1, y_loc,...
    '$\omega_1$\hspace{1pt}','HorizontalAlignment','right',...
    'VerticalAlignment','bottom');
h = text(w2, y_loc,...
    '\hspace{2pt}$\omega_2$','HorizontalAlignment','left',...
    'VerticalAlignment','bottom');
h = text(wp_est, y_loc,...
    '\hspace{2pt}$\omega_\mathrm{sc}$','HorizontalAlignment','left',...
    'VerticalAlignment','bottom');
xticklabels([])

nexttile(1)
l = legendLatex(fig,name); 
set(l,'location','southwest','Orientation','vertical')

saveLatex(fig,'Bode_lin_sync_loop',fig_save_opt)