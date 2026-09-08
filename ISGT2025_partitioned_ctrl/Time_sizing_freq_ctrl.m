opt_ISGT2025

[PAR, opt] = data_Nordic5(opt); % Modell 

ng = PAR.ng;


%% Dimensioning frequency event
k = (1400)/0.4 -400; % Static gain [MW/Hz];
Ftot = k*(6.5*s+1)/(2*s+1)/(17*s+1);
Mtot = 2*110e3/50;
Dtot = 400;
Gtot = minreal(1/(s*Mtot+Ftot+Dtot));
opt.PODP_input_func_filter = eye(PAR.nbus)*Gtot;

opt2 = opt;
opt2.PODP = opt2.PODP/2;

[PAR, opt] = data_Nordic5(opt); % Modell 
[PAR2, opt2] = data_Nordic5(opt2); % Modell 
% step(Gtot*1400)
%
idx = 6;
simopt = struct();
simopt.dPL = zeros(PAR.nbus,1); % Störning aktiv effekt
simopt.dPL(2) = [0]; % Störning i per unit, P_bas = PAR.Sb
dP = zeros(PAR.nbus,1);

% simopt.dPL(3) = 1400/PAR.Sb; % Störning i per unit, P_bas = PAR.Sb
dP(cbus(idx)) = 1400/1;

step_func = @(T,dy) dP.*((T>1));

Tend = 20;
t=Tend;

% dP(1) = 0.01*PAR.Sb;
% step_func = @(T) dP.*(T>5);
PAR.PODP.input_func = step_func; % Input function
PAR.PODP.CL = ones(PAR.nbus,1);
PAR.PODP.CL(cbus(idx)) = 0; % <------------- Open POD-P control loop for the test
PAR.PODQ.CL = PAR.PODP.CL;
y1 = runsim(PAR, t, simopt);

PAR2.PODP.input_func = step_func; % Input function
PAR2.PODP.CL = ones(PAR2.nbus,1);
PAR2.PODP.CL(cbus(idx)) = 0; % <------------- Open POD-P control loop for the test
PAR2.PODQ.CL = PAR2.PODP.CL;
y2 = runsim(PAR2, t, simopt);


[fig1,~,co] = figureLatex(4,4/sqrt(2));
tile1= tiledLatex(2,1);
nexttile(1)
plot(y1.TIME,-y1.u_FREQ_PODP_test(:,cbus(idx))+49.9,'color',[1,1,1]*0); hold all
plot(y1.TIME,y1.u_OMEGAC(:,idx)-0.1,'color',co(1,:)); hold all
plot(y2.TIME,y2.u_OMEGAC(:,idx)-0.1,'--','color',co(1,:)); hold all
% plot(y2.TIME,-y2.u_FREQ_PODP_test(:,cbus(idx))+49
% .9); hold all
ylabel('Frequency, $\omega_c$ [Hz]')
xlim([0,Tend])
xticks([0,5,10,15,20])
xticklabels([])

nexttile(2)
yyaxis left
plot(y1.TIME,y1.Id_CONV(:,idx)/PAR.CONV.MBASE(idx),'color',[1,1,1]*0); hold all
plot(y2.TIME,y2.Id_CONV(:,idx)/PAR.CONV.MBASE(idx),'--','color',[1,1,1]*0); hold all

% plot(y1.TIME,abs(y1.Id_CONV(:,idx)+1j*y1.Iq_CONV(:,idx) )/PAR.CONV.MBASE(idx),'color',[1,1,1]*0); hold all
% plot(y2.TIME,abs(y2.Id_CONV(:,idx)+1j*y2.Iq_CONV(:,idx) )/PAR.CONV.MBASE(idx),'--','color',[1,1,1]*0); hold all
ax =gca;
set(ax,'Ycolor',[1,1,1]*0.15)
ylabel('Current, $I_{d}$ [p.u.]')
ylims1 = ylim;
yyaxis right
plot(y1.TIME,y1.Iq_CONV(:,idx)./PAR.CONV.MBASE(idx),'-','color',co(1,:)); hold all
plot(y2.TIME,y2.Iq_CONV(:,idx)./PAR.CONV.MBASE(idx),'--','color',co(1,:)); hold all
ax =gca;
set(ax,'Ycolor',co(1,:))
ylabel('Current, $I_q$ [p.u.]')
xlim([0,Tend])
ylims2 = ylim;
yscale = abs(diff(ylims1)/diff(ylims2));
ylim(0.1-[abs(diff(ylims1)),0]) 

l = legendLatex(fig1,{'$k_F = 2$\,p.u./Hz','$k_F = 1$\,p.u./Hz'}); 
set(l,'location','southeast','Orientation','vertical')

xlabel('Time [s]')

nexttile(1)
l = legendLatex(fig1,{'$\omega_c^*$ (test signal)','$\omega_c$'}); 
% l = legendLatex(fig1,{'$\omega_c^*$ (test signal)','$\omega_c$'}); 
set(l,'location','southeast','Orientation','vertical')

saveLatex(fig1,'Time_sizing',fig_save_opt)

