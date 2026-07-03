opt_HVDC_wind

% opt.PODP = opt.PODP*1;
% opt.PODP = pade(opt.PODP*exp(-s*0.1),6);
% 
% opt.PODP(7,7) = opt.PODP(7,7)*1;
% opt.PODP(8,8) = opt.PODP(8,8)*0;
% opt.PODQ = opt.PODQ*1;
% [PAR, opt] = data_Nordic5(opt); % Modell 


runsimulation = 1; %% <-- Skip simulation to speed things up. Only if data is already in workspace



% opt.CONV.MBASE = 1*ones(m,1); % Lower the rating to get more visible difference between ESCP and SCP
% opt.CONV.Storage = ss(0*eye(m));
[PAR, opt] = data_Nordic5(opt); % Modell 

simopt = struct();
idx = 1; %
% PAR.CONV.sync_ctrl.sys = PAR.CONV.sync_ctrl.sys*10000;

PAR.PODP.CL = ones(PAR.nbus,1);
PAR.PODP.CL(cbus(idx)) = 0; % <------------- Open POD-P control loop for the test



period_times = [100,40,10,4,3.5,3,2.5,2,1.5,1.0,0.8,0.6,0.4,0.2,0.1,0.01,0.001];
period_times = [40,10,4,3,2,1.0,0.5,0.1];
% period_times = [40,10,5,2.5,1.0];
% period_times = [2.5,1.0];
periods_per_level = 10;
skip_periods = 4;
T = sum(periods_per_level*period_times); % All frequencies will be performed in the same simulation
tt = period_times(1)/3;

amp = zeros(PAR.nbus,1);
amp(cbus(idx)) = 1e-3; % Hz
sweep_func = @(T,x) frequency_sweep_signal(T,amp,period_times,periods_per_level)*(T/tt*(T<tt) + (T>tt));
PAR.PODP.input_func = sweep_func; % Input function
if runsimulation == 1% <----------------- Skip simulation to speed things up  
    y1 = runsim(PAR,T,simopt);
    u1 = y1.u_CONV_PQ(:,idx);
    [f1,gain1,phase1] = fft_of_frequency_sweep(y1.TIME,u1/amp(cbus(idx)),period_times,periods_per_level,skip_periods);
    
end
u_ref = [];
for i = 1:length(y1.TIME)
    u_ref = [u_ref,PAR.PODP.input_func(y1.TIME(i))];
end
PAR.PODP = rmfield(PAR.PODP,'input_func');

PAR.PODP.bypass_input_func = sweep_func; % Input function
if runsimulation == 1% <----------------- Skip simulation to speed things up   
    y2 = runsim(PAR,T,simopt);
    u2 = y2.u_CONV_PQ(:,idx);
    [f2,gain2,phase2] = fft_of_frequency_sweep(y2.TIME,u2/amp(cbus(idx)),period_times,periods_per_level,skip_periods);
end
PAR.PODP = rmfield(PAR.PODP,'bypass_input_func');

% amp = zeros(m,1);
% amp(idx) = 0.01/PAR.CONV.MBASE(idx);
% sweep_func = @(T,x) frequency_sweep_signal(T,amp,period_times,periods_per_level)*(T/tt*(T<tt) + (T>tt));
% PAR.CONV.P_input_func = sweep_func; % Input function
% if runsimulation == 1% <----------------- Skip simulation to speed things up 
%     y3 = runsim(PAR,T,simopt);
%     u3 = y3.u_CONV_PQ(:,idx);
%     [f3,gain3,phase3] = fft_of_frequency_sweep(y3.TIME,u3/amp(idx),period_times,periods_per_level,skip_periods);
% end
% PAR.CONV = rmfield(PAR.CONV,'P_input_func');

%% Plot frequency sweep time series
[fig2,~,co] = figureLatex(4,2.5/sqrt(2));
tiledLatex(1,1);

u_ref = u_ref(cbus(idx),:);
yyaxis left
plot(y1.TIME,u_ref*1000,'color',[1,1,1]*0); hold all
ax =gca;
set(ax,'Ycolor',[1,1,1]*0.15)
% ytickformat('%,.2e')
ylabel(['Frequency error, $\omega_\mathrm{err}$ [mHz] $\quad$'])
yyaxis right
plot(y1.TIME(:),u1/PAR.CONV.MBASE(idx),'color',co(1,:)); hold all
ylabel(['Active power, $P$ [p.u.]'])
ax =gca;
set(ax,'Ycolor',co(1,:))


N = length(period_times); % Number of frequencies simulated
time_per_level = period_times*periods_per_level;
A = ones(N); A = triu(A);
A = A - eye(N);
time_at_start_of_level = time_per_level*A;
xline(time_at_start_of_level(2:end),'linewidth',1.5)

xlabel('Time [s]')
xlim([y1.TIME(1),y1.TIME(end)])

saveLatex(fig2,'Time_sweep_PODP_loop',fig_save_opt)

%% Bode Plot
PAR.input = 'PODP_OMEGA_ref'; % Input in Hz
PAR.output = 'PQ_CONV'; % Output in pu
G = lin_fg_xy(PAR.xy0,PAR);
Pdc = PAR.CONV.MBASE(idx);
% Gp = G(idx,cbus(idx))/(2*pi)/Pdc; % Omega_err [rad/s] --> Power [pu] (converter base)
Gp = G(idx,cbus(idx))/Pdc; % Omega_err [Hz] --> Power [pu] (converter base)

PAR.input = 'bypass_PODP'; % Input in pu
PAR.output = 'PQ_CONV'; % Output in pu
G = lin_fg_xy(PAR.xy0,PAR);
T_HVDC_wind = G(idx,cbus(idx)); % Power [pu] --> Power [pu] (same base)

PAR.input = 'Pref_CONV';
PAR.output = 'PQ_CONV';
G = lin_fg_xy(PAR.xy0,PAR); 
T = G(idx,idx); % Power [pu] --> Power [pu] (same base)

ESCP = opt.data.ESCP(idx); % Short circuit power [pu] (system base)
ESCR = ESCP/Pdc; % Short circuit ratio 


[fig3,~,co] = figureLatex(3.5,3.5/sqrt(2));

bodeopt = struct();
bodeopt.wrap = true;
% bodeopt.adjust = false;
% bodeopt.FreqUnits = 'Hz';
bodeopt.omega_lim = [1e-2,1e4] ;

bodeopt = bodeLatex(Gp,bodeopt); hold all

w1 = 0.25*2*pi;
w2 = 1.0*2*pi;
bode_draw_window3(w1,w2,-45,45,co(5,:))
% wc_est = sqrt(ESCP/M);
b = 0.2;

ki = PAR.CONV.ki_sync(idx);
wc_est = sqrt(ESCP*ki);
z = 5.8*10*1e-3;
wd = 0.1;
for i = 1:2
    nexttile(i)
    xline(w1); hold all
    xline(w2); hold all
    xline(z); hold all
    xline(wd); hold all
    % xline(b); hold all
    % xline(wc_est); hold all
    grid on
end

% bodeopt.color = co(5,:);
% bodeopt = bodeLatex(ESCR/s,bodeopt); hold all
bodeopt.color = co(3,:);
bodeopt = bodeLatex(T_HVDC_wind,bodeopt); hold all
bodeopt.color = co(1,:);
bodeopt = bodeLatex(Gp,bodeopt); hold all

% bodeopt.color = [1,1,1]*0.5;
% bodeopt.linestyle = '--';
% bodeopt = bodeLatex(T,bodeopt); hold all



nexttile(1)
loglog(f2*2*pi,gain2,'x', 'color',co(3,:)); hold all
loglog(f1*2*pi,gain1/Pdc,'x', 'color',co(1,:)); hold all

% loglog(f3*2*pi,gain3,'x', 'color',[1,1,1]*0.5); hold all
nexttile(2)
semilogx(f2*2*pi,phase2,'x', 'color',co(3,:)); hold all
semilogx(f1*2*pi,phase1,'x', 'color',co(1,:)); hold all

set(gca, 'ytick', (-180:90:180));
ylim([-180,180])

nexttile(1);
ylim([0.5*10^-2,10^1])
set(gca, 'ytick', 10.^(-10:2:10));
xticklabels([])

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
% h = text(b, y_loc,...
%     '$\omega_E$\hspace{1pt}','HorizontalAlignment','right',...
%     'VerticalAlignment','bottom');
% h = text(wc_est, y_loc,...
%     '\hspace{2pt}$\omega_\mathrm{sc}$','HorizontalAlignment','left',...
%     'VerticalAlignment','bottom');
%
nexttile(2);
ax = gca;
line_handle = get(ax, 'Children');
l = legendLatex(fig3,{'$P/\omega_\mathrm{err}$ [p.u./Hz]','$P/P_F$'},line_handle([3,4])); 
set(l,'location','northeast','Orientation','vertical')

saveLatex(fig3,'Bode_sweep_PODP_loop',fig_save_opt)

