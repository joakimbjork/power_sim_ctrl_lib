%%
opt_HVDC_wind
% opt.CONV.MBASE = 1*ones(m,1); % Lower the rating to get more visible difference between ESCP and SCP
% opt.CONV.Storage = ss(0.1*eye(m));
% 
% opt.PODP = opt.PODP*1;
% opt.PODP = pade(opt.PODP*exp(-s*0.1),6);
% opt.PODP(7,7) = opt.PODP(7,7)*1;
% opt.PODP(8,8) = opt.PODP(8,8)*0;
opt.PODP = opt.PODP*1;
opt.PODQ = opt.PODQ*1;
[PAR, opt] = data_Nordic5(opt); % Modell 



Tend = 100;
% t = [1,1.05,Tend];
t = [1,Tend];
d_bus = [1]; % Välj var störningen ska ske

simopt = struct();
simopt.dPL = zeros(PAR.nbus,length(t)); % Störning aktiv effekt
simopt.dPL(d_bus,2:end) = [14]; % Störning i per unit, P_bas = PAR.Sb
% simopt.dPL(2,3:end) = [6]; % Störning i per unit, P_bas = PAR.Sb
% simopt.dQL0 = zeros(PAR.nbus,1); % Störning aktiv effekt

% simopt.dQL0 = zeros(PAR.nbus,length(t)); % Störning aktiv effekt
% simopt.dQL0(14,2) = [14]; % Störning i per unit, P_bas = PAR.Sb
% simopt.dQL(14,3) = [2]; % Störning i per unit, P_bas = PAR.Sb

% 
% simopt = struct();
% simopt.dPL = zeros(PAR0.nbus,length(t)); % Störning aktiv effekt
% simopt.dPL(d_bus,3) = [9]; % Störning i per unit, P_bas = PAR.Sb
% simopt.dPL(d_bus,2) = [9*1e-3]; % Störning i per unit, P_bas = PAR.Sb
% 
% dP = zeros(length(cbus),1);
% Tramp = 1;
% dP(1) = 1*PAR.Sb/Tramp;
% dP(2) = dP(1);
% step_func = @(T) dP.*((T>1)*(T-1) - (T>(1+Tramp))*(T-(1+Tramp)));
% step_func = @(T) dP.*((T>1));
% % dP(1) = 0.01*PAR.Sb;
% % step_func = @(T) dP.*(T>5);
% PAR.CONV.P_input_func = step_func; % Input function

% y0 = runsim(PAR0, t, simopt);
y1 = runsim(PAR, t, simopt);
%%
time_window = [0,Tend];
% time_window = [0,10];
nc = length(cbus);
[fig1,~,co] = figureLatex(7/0.8,4/sqrt(2));
tile1 = tiledLatex(2,4);
nexttile(1)
% plot(y1.TIME,y1.u_OMEGAC-0.1); hold all
plot(y1.TIME,y1.OMEGA-0.1); hold all
ylabel('Frequency, $\omega$ [Hz]')
xlim(time_window)
% ylim([48.5,50])

nexttile(5)
% plot(y1.TIME,(y1.u_PM)*PAR.Sb,'color',[1,1,1]*0.7); hold all 
plot(y1.TIME,(y1.u_CONV_PQ(:,1:nc))*PAR.Sb); hold all
plot(y1.TIME,(y1.u_P_PODP(:,PAR.CONV.Cbus)+y1.u_CONV_PQ(1,1:nc))*PAR.Sb,'--'); hold all
ylabel('Power, $P$ [MW]')
xlim(time_window)
% xticklabels([])


nexttile(2)
plot(y1.TIME,y1.HVDC_wind_Vdc); hold all
ylabel('Voltage, $V_\mathrm{DC}$ [p.u.]')
xlim(time_window)

nexttile(3)
plot(y1.TIME,y1.HVDC_wind_rotor_speed); hold all
ylabel('Rotor speed, $\Omega_\mathrm{nom}$ [p.u.]')
xlim(time_window)

nexttile(6)
plot(y1.TIME,y1.HVDC_wind_P1*PAR.Sb); hold all
ylabel('Power, $P_1$ [MW]')
xlim(time_window)

nexttile(7)
plot(y1.TIME,y1.HVDC_wind_Pwind*PAR.Sb); hold all
ylabel('Power, $P_\mathrm{e,wind}$ [MW]')
xlim(time_window)

nexttile(4)
plot(y1.TIME,y1.HVDC_wind_dP_K1*PAR.Sb); hold all
ylabel('Ref, $dP_1$ [MW]')
xlim(time_window)

nexttile(8)
plot(y1.TIME,y1.HVDC_wind_dP_K2*PAR.Sb); hold all
% plot(y1.TIME,y1.u_P_PODP(:,PAR.CONV.Cbus)*PAR.Sb,'--'); hold all
ylabel('Ref, $dP_2$ [MW]')
xlim(time_window)

%%
time_window = [0,45];
% time_window = [0,10];
nc = length(cbus);
[fig1,~,co] = figureLatex(7/0.8,4/sqrt(2));
tile1 = tiledLatex(2,3);
nexttile(1)
% plot(y1.TIME,y1.u_OMEGAC-0.1); hold all
plot(y1.TIME,y1.OMEGA-0.1); hold all
ylabel('Machine speed [Hz]')
xlim(time_window)
% ylim([48.5,50])

nexttile(2)
plot(y1.TIME,y1.HVDC_wind_Vdc); hold all
ylabel('Voltage, $V_\mathrm{DC}$ [p.u.]')
xlim(time_window)

nexttile(3)
plot(y1.TIME,y1.HVDC_wind_rotor_speed+1); hold all
ylabel('Rotor speed [p.u.]')
xlim(time_window)

nexttile(5)
% plot(y1.TIME,(y1.u_PM)*PAR.Sb,'color',[1,1,1]*0.7); hold all 
plot(y1.TIME,(y1.u_CONV_PQ(:,1:nc))*PAR.Sb); hold all
plot(y1.TIME,(y1.u_P_PODP(:,PAR.CONV.Cbus)+y1.u_CONV_PQ(1,1:nc))*PAR.Sb,'--'); hold all
ylabel('Power, $P_2$ [MW]')
if nc ==1
l = legendLatex(fig1,{'$P_2$','$P_F$'}); 
set(l,'location','northeast','Orientation','vertical')
end
xlim(time_window)
% xticklabels([])


nexttile(4)
plot(y1.TIME,y1.HVDC_wind_P1*PAR.Sb); hold all
ylabel('Power, $P_1$ [MW]')
xlim(time_window)

nexttile(6)
plot(y1.TIME,(y1.u_CONV_PQ(:,nc+(1:nc)))*PAR.Sb); hold all
% plot(y1.TIME,y1.VOLT); hold all
ylabel('Reactive power [MVar]')
xlim(time_window)


