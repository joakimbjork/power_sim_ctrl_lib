opt_CONV_study
% opt.CONV.MBASE = 1*ones(m,1); % Lower the rating to get more visible difference between ESCP and SCP
% opt.CONV.Storage = ss(0.1*eye(m));
% 
opt.PODP = opt.PODP*1;
opt.PODQ = opt.PODQ*1;
[PAR, opt] = data_Nordic5(opt); % Modell 

opt0 = opt;
opt0.PODP = opt0.PODP*0;
opt0.PODQ = opt0.PODQ*1;
[PAR0, opt0] = data_Nordic5(opt0); % Modell 

Tend = 40;
% t = [1,1.05,Tend];
t = [1,Tend];
d_bus = [14]; % Välj var störningen ska ske

simopt = struct();
simopt.dPL = zeros(PAR.nbus,length(t)); % Störning aktiv effekt
simopt.dPL(d_bus,2:end) = [14]; % Störning i per unit, P_bas = PAR.Sb


y0 = runsim(PAR0, t, simopt);
y1 = runsim(PAR, t, simopt);
%
% plot3_freq_control;
% plot4_freq_volt;

%%
cbus = PAR.CONV.Cbus;
% cbus = [8,9,11,12,13,14]; % Closest node [2,2,6,6,5,1]
cbus_reorder = [6,2,1,5,3,4];

[fig1,~,co] = figureLatex(7/0.8,4/sqrt(2));
tile1 = tiledLatex(2,3);
nexttile(1)
plot(y0.TIME,y0.u_OMEGAC-0.1,'color',[1,1,1]*0.75); hold all
for i = linspace(6,1,6)
    ii = cbus_reorder(i);
    plot(y1.TIME,y1.u_OMEGAC(:,ii)-0.1,'color',co(i,:)); hold all
end
% plot(y1.TIME,y1.u_OMEGAC(:,cbus_reorder)-0.1); hold all
% plot(y1.TIME,y1.u_OMEGAC(:,6)-0.1,'k'); hold all
ylabel('Frequency, $\omega_c$ [Hz]')
xlim([0,Tend])
ylim([48.5,50])

% ax = gca;
% line_handle = get(ax, 'Children');
% l = legendLatex(fig1,{'$F(s)=0$','$F(s)\neq 0$'},line_handle([7,1])); 
% set(l,'location','southeast','Orientation','vertical')

% xticks([0,5,10,15,20])
xticklabels([])


nexttile(4)
plot(y0.TIME,y0.VOLT(:,cbus(cbus_reorder)),'color',[1,1,1]*0.75); hold all
for i = linspace(6,1,6)
    ii = cbus_reorder(i);
    plot(y1.TIME,y1.VOLT(:,cbus(ii)),'color',co(i,:)); hold all
end
% plot(y1.TIME,y1.VOLT(:,cbus(cbus_reorder))); hold all
% plot(y1.TIME,y1.VOLT(:,cbus(6)),'k'); hold all
ylabel(' Voltage, $|V_{dq}|$ [p.u.]')
xlim([0,Tend])
xlabel('Time [s]')


% [fig2,~,co] = figureLatex(4,4);
% tile2 = tiledLatex(2,1);
nexttile(2)
% plot(y1.TIME,(y1.u_PM)*PAR.Sb,'color',[1,1,1]*0.7); hold all 
plot(y0.TIME,(y0.u_CONV_PQ(:,cbus_reorder))*PAR.Sb,'color',[1,1,1]*0.75); hold all
for i = linspace(6,1,6)
    ii = cbus_reorder(i);
    plot(y1.TIME,(y1.u_CONV_PQ(:,ii))*PAR.Sb,'color',co(i,:)); hold all   
end
ylabel('Power, $P$ [MW]')
xticklabels([])


% MWs_to_kWh = 60*60*1e-3; 
nexttile(5)
plot(y0.TIME,(y0.u_CONV_PQ(:,cbus_reorder+length(cbus)))*PAR.Sb,'color',[1,1,1]*0.75); hold all
for i = linspace(6,1,6)
     ii = cbus_reorder(i)+length(cbus);
    plot(y1.TIME,(y1.u_CONV_PQ(:,ii))*PAR.Sb,'color',co(i,:)); hold all 
end
ylabel('Power, $Q$ [Mvar]')
xlabel('Time [s]')
ylim([-50,200])


nexttile(3)
% plot(y1.TIME,(y1.u_PM)*PAR.Sb,'color',[1,1,1]*0.7); hold all 

% plot(y0.TIME,(y0.u_CONV_PQ(:,cbus_reorder))./PAR.CONV.MBASE(cbus_reorder)','color',[1,1,1]*0.7); hold all
plot(y0.TIME,abs(y0.Id_CONV(:,cbus_reorder)+1j*y0.Iq_CONV(:,cbus_reorder))./PAR.CONV.MBASE(cbus_reorder)','color',[1,1,1]*0.75); hold all
% plot(y0.TIME,abs(y0.Id_CONV(:,cbus_reorder))./PAR.CONV.MBASE(cbus_reorder)','color',[1,1,1]*0.7); hold all
for i = linspace(6,1,6)
    ii = cbus_reorder(i);
    % plot(y1.TIME,(y1.u_CONV_PQ(:,ii))./PAR.CONV.MBASE(ii),'color',co(i,:)); hold all   
    plot(y1.TIME,abs(y1.Id_CONV(:,ii)+1j*y1.Iq_CONV(:,ii))/PAR.CONV.MBASE(ii),'color',co(i,:)); hold all  
end
% ylabel('Power, $P$ [pu]')
ylabel('Current, $|I_{dq}|$ [p.u.]')
xticklabels([])

yline(1)
yline(1.05)
ylim([0.35,1.2])

h = text(10, 1,...
'$I_\mathrm{max}''=1$\,p.u.','HorizontalAlignment','left',...
'VerticalAlignment','top');

h = text(2, 1.05,...
'$I_\mathrm{max}''''=1.05$\,p.u.','HorizontalAlignment','left',...
'VerticalAlignment','bottom');

% nexttile(6)
% plot(y0.TIME,y0.u_CONV_Echarge(:,cbus_reorder)./PAR.CONV.MBASE(cbus_reorder)','color',[1,1,1]*0.75); hold all
% % plot(y0.TIME,y0.u_CONV_Echarge(:,cbus_reorder)*PAR.Sb/(60*60),'color',[1,1,1]*0.75); hold all
% for i = linspace(6,1,6)
%      ii = cbus_reorder(i);
% plot(y1.TIME,y1.u_CONV_Echarge(:,ii)./PAR.CONV.MBASE(ii),'color',co(i,:)); hold all
% % plot(y1.TIME,y1.u_CONV_Echarge(:,ii)*PAR.Sb/(60*60),'color',co(i,:)); hold all
% end
% ylabel('Energy [pu$\times$s]')
% xlabel('Time [s]')

nexttile(6)
% plot(y0.TIME,y0.u_CONV_Echarge(:,cbus_reorder)./PAR.CONV.MBASE(cbus_reorder)','color',[1,1,1]*0.75); hold all
plot(y0.TIME,y0.u_CONV_Echarge(:,cbus_reorder)*PAR.Sb/(60*60)*1000,'color',[1,1,1]*0.75); hold all
for i = linspace(6,1,6)
     ii = cbus_reorder(i);
% plot(y1.TIME,y1.u_CONV_Echarge(:,ii)./PAR.CONV.MBASE(ii),'color',co(i,:)); hold all
plot(y1.TIME,y1.u_CONV_Echarge(:,ii)*PAR.Sb/(60*60)*1000,'color',co(i,:)); hold all
end
ylabel('Energy [kWh]')
xlabel('Time [s]')

nexttile(1)
ax = gca;
line_handle = get(ax, 'Children');
l = legendLatex(fig1,{'$F(s)=0$','$F(s)\neq 0$'},line_handle([7,1])); 
set(l,'location','southeast','Orientation','vertical')

nexttile(4)
ax = gca;
line_handle = get(ax, 'Children');
l = legendLatex(fig1,{'Bus 11','Bus 211','Bus 212','Bus 51','Bus 611','Bus 612'},line_handle([1,2,3,4,5,6])); 
set(l,'location','southeast','Orientation','vertical','NumColumns', 2)
% set(l,'location','none','Orientation','horizontal')
% l.Layout.Tile = 'middle';

saveLatex(fig1,'Time_load_dist',fig_save_opt)
% nexttile(1)
% plot(y1.TIME,abs(y1.Id_CONV./PAR.CONV.MBASE'+1j*y1.Iq_CONV./PAR.CONV.MBASE'),'color',co(1,:)); hold all
% ylabel('$E_d$ [pu]')
% xlim([0,Tend])
%%

[fig3,~,co] = figureLatex(4,2/sqrt(2));
% tile3 = tiledLatex(1,1);
% nexttile(1)
ii = cbus_reorder(1);

plot(y1.TIME,(y1.Id_CONV(:,ii)/PAR.CONV.MBASE(ii)),'color',co(1,:)); hold all
plot(y1.TIME,abs(y1.Id_CONV(:,ii)+1j*y1.Iq_CONV(:,ii))/PAR.CONV.MBASE(ii),'color',[1,1,1]*0); hold all  

xlims = xlim;
yline(1)
yline(1.05)
x_loc = xlims(2);
h = text(28, 1,...
'$I_\mathrm{max}''=1$\,p.u.','HorizontalAlignment','left',...
'VerticalAlignment','top');

h = text(28, 1.05,...
'$I_\mathrm{max}''''=1.05$\,p.u.','HorizontalAlignment','left',...
'VerticalAlignment','bottom');

ylabel('Current [pu]')
l = legendLatex(fig3,{'$I_d$', '$|I_{dq}|$'}); set(l,'location','southeast')

ylim([0.65,1.15])
xlabel('Time [s]')
% xlim([0.99,1.1])
saveLatex(fig3,'Time_load_dist_current',fig_save_opt)

%%
[fig2,~,co] = figureLatex(4,4);
tile2 = tiledLatex(3,1);
nexttile(1)
plot(y1.TIME,y1.u_DELTAC-y1.ANG(:,PAR.CONV.Cbus)); hold all
% plot(y.TIME,y.ANG(:,PAR.CONV.Cbus),'--'); hold all    
ylabel('Phase angle ($\delta-\theta$) [deg]')
xlim([0,Tend])

nexttile(2)
plot(y1.TIME,y1.u_OMEGAC-0.1); hold all
ylabel('Freq [Hz]')
xlim([0,Tend])

nexttile(3)
plot(y1.TIME,y1.VOLT(:,[1,14])); hold all
ylabel('PCC Voltage [p.u.]')
xlim([0,Tend])
%
[fig3,~,co] = figureLatex(6,4);
tile3 = tiledLatex(3,2);
nexttile(1)
plot(y1.TIME,y1.Ed_CONV); hold all
ylabel('$E_d$ [p.u.]')
xlim([0,Tend])

nexttile(2)
plot(y1.TIME,y1.Eq_CONV); hold all    
ylabel('$E_q$ [p.u.]')
xlim([0,Tend])

nexttile(3)
plot(y1.TIME,y1.Id_CONV./PAR.CONV.MBASE'); hold all
ylabel('$I_d$ [p.u.]')
xlabel('Time [s]')
xlim([0,Tend])

nexttile(4)
plot(y1.TIME,y1.Iq_CONV./PAR.CONV.MBASE'); hold all 
ylabel('$I_q$ [p.u.]')
xlabel('Time [s]')
xlim([0,Tend])

nexttile(5)
if isfield(y1,'u_CONV_PQ')
    for i = 1:length(PAR.CONV.Cbus)
        names{i} = ['CONV ', num2str(PAR.CONV.Cbus(i))];
    end    
    plot(y1.TIME,(y1.u_CONV_PQ(:,1:length(PAR.CONV.Cbus)))./PAR.CONV.MBASE'); hold all   
end
yline(0)
l = legendLatex(fig3,[names]); set(l,'location','east')
ylabel('Power, $\Delta P$ [p.u.]')

nexttile(6)
if isfield(y1,'u_CONV_PQ')
    for i = 1:length(PAR.CONV.Cbus)
        names{i} = ['CONV ', num2str(PAR.CONV.Cbus(i))];
    end    
    plot(y1.TIME,(y1.u_CONV_PQ(:,length(PAR.CONV.Cbus)+(1:length(PAR.CONV.Cbus))))./PAR.CONV.MBASE'); hold all   
end
yline(0)
l = legendLatex(fig3,[names]); set(l,'location','east')
ylabel('Reactive power, $\Delta Q$ [p.u.]')
