
[fig,names,co] = figureLatex(5,5);
if isfield(opt.CONV, 'Storage')
    tile = tiledLatex(3,1); 
else
    tile = tiledLatex(2,1);
end
title(tile,['Load disturbance (', num2str(sum(simopt.dPL)*PAR0.Sb) ,' MW) at bus ', num2str(d_bus)])

nexttile(1)
plot(y1.TIME,y1.OMEGA); hold all
ylabel('Rotor speed, $\omega$ [Hz]')
xlim([0,Tend])
l = legendLatex(fig,{'GEN 1','GEN 2','GEN 3','GEN 4','GEN 5'}); set(l,'location','southeast')
% ylim([49.4,50.1])

nexttile(2)
if isfield(y1,'u_CONV_PQ')
    plot(y1.TIME,(y1.u_PM(:,1:3)-y1.u_PM(1,1:3))*PAR0.Sb); hold all
    plot(y1.TIME,(y1.u_CONV_PQ(:,1:2))*PAR0.Sb); hold all
    yline(0)
l = legendLatex(fig,{'GEN 1','GEN 2','GEN 3','CONV 5','CONV 6'}); set(l,'location','east')
else
    plot(y1.TIME,(y1.u_PM-y1.u_PM(1,:))*PAR0.Sb); hold all
    yline(0)
    l = legendLatex(fig,{'GEN 1','GEN 2','GEN 3','GEN 4','GEN 5'}); set(l,'location','east')
end
ylabel('Power, $\Delta P$ [MW]')
if isfield(opt.CONV, 'Storage')
    nexttile(3)
    plot(y1.TIME,y1.u_CONV_Echarge*PAR0.Sb); hold all
    l = legendLatex(fig,{'CONV 5','CONV 6'}); set(l,'location','east')
    ylabel('Energy, $\Delta E_\mathrm{charge}$ [MWs]')
end

if ~isequal(PAR0.CONV.Cbus,[5,6])
    warning('Incorrect labeling of converter buses')
end

xlim([0,Tend])


xlabel('Time [s]')