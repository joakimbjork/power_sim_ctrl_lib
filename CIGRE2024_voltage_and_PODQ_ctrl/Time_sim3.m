opt_CIGRE2024
PAR_db = data_PODQ_test_system(opt_db_default);

simopt = struct();
simopt.show_active_power_flow = true;

simopt_load_trip = simopt;
simopt_load_trip.dPL = -[0.25;-0.25;0;0]; % Fixed size disturbance at bus 1 and 2
simopt_dist1 = simopt_load_trip;
simopt_load_trip.dPL = -[0.20;-0.20;0;0]; % Fixed size disturbance at bus 1 and 2
simopt_dist2 = simopt_load_trip;

Tend =  80;
% Tend = [1,2,50];
dVref = 0*0.04*1;

% simopt_dist = simopt_load_trip;

step_func = @(T,u) dVref*ones(4,1).*(T>1); % Step function
sine_func = @(T,u) 0*ones(4,1).*sin(2*pi*2*T); % sine function
PAR_db.PODQ.input_func = sine_func; % input function?
PAR_db.PODQ.CL = 1;

y = cell(0);
[y{1}] = runsim(PAR_db,Tend,simopt_dist1);
[y{2}] = runsim(PAR_db,Tend,simopt_dist2);
% [y{3}] = runsim(PAR_PODQ,Tend,simopt_dist2);

[fig1,~,co] = figureLatex(4,3.5);
tile1 = tiledLatex(2,1);

T = 80;
[fig2,~,co] = figureLatex(4,3.5);
tile2 = tiledLatex(2,1);
% title(tile2,['Rotor angle and power flow'])

for idx = 1:2
    y_ = y{idx};

    % downsample
    n_sample = 40;
    t_length = length(y_.TIME);
    t_idx = false(t_length,1);
    t_idx(1:n_sample:t_length)=true;
    t_idx(end) = true;
    t_idx(y_.TIME==1) = true;

    col = co(idx,:);
    style = '-';

    figure(fig1)
    nexttile(1)
    plot(y_.TIME(t_idx),y_.VOLT(t_idx,bus_idx),style,'color',col); hold all
    ylabel('Voltage, $V_4$ [p.u.]')
    xlim([0,T])
        
    nexttile(2)
    plot(y_.TIME(t_idx),-y_.u_QL(t_idx,bus_idx),style,'color',col); hold all
    ylabel('Reactive power, $Q_4$ [p.u.]')
    xlim([0,T])
    xlabel('Time [s]')

    figure(fig2)
    nexttile(1)
    plot(y_.TIME(t_idx),y_.DELTA(t_idx,1)-y_.DELTA(t_idx,2),style,'color',col); hold all
    ylabel('Rotor angle, $\delta_1-\delta_2$ [$^\circ$]')
    xlim([0,T])

    nexttile(2)
    from = 1; to = 2;
    plot(y_.TIME(t_idx),y_.Pline(t_idx, PAR.ng+PAR.nbus*(to-1)+from ),style,'color',col); hold all
    xlim([0,T])
    ylabel(['Active power, $P_\mathrm{line}$ [p.u.]'])
    xlabel('Time [s]')
end

% close(fig2)
figure(fig1)
nexttile(1)


% xticks(x_axis_ticks)
xticklabels([])

Vstart = PAR_voltage.U0(4);
pick_out = zeros(1,4); pick_out(bus_idx) = 1;
% plot(y{1}.TIME,Vstart+pick_out*PAR_db.PODQ.input_func(y{1}.TIME',1),'--','color',[1,1,1]*0.6)
p2 = [];

p2(1) = plot([0,T],Vstart+db*1.3*[1,1],'-','color',[1,1,1]*0.6);
p2(2) = plot([0,T],Vstart+db*1.0*[1,1],'-','color',co(3,:));
plot([0,T],Vstart-db*1.0*[1,1],'-','color',co(3,:))
plot([0,T],Vstart-db*1.3*[1,1],'-','color',[1,1,1]*0.6)

% p2(1) = plot(y{1}.TIME,Vstart+db*1.3+pick_out*PAR_db.PODQ.input_func(y{1}.TIME',1),'-','color',[1,1,1]*0.6);
% p2(2) = plot(y{1}.TIME,Vstart+db+pick_out*PAR_db.PODQ.input_func(y{1}.TIME',1),'-','color',co(3,:));
% plot(y{1}.TIME,Vstart-db+pick_out*PAR_db.PODQ.input_func(y{1}.TIME',1),'-','color',co(3,:))
% plot(y{1}.TIME,Vstart-db*1.3+pick_out*PAR_db.PODQ.input_func(y{1}.TIME',1),'-','color',[1,1,1]*0.6)
% ylim(Vstart+[-1,1]*3*db)
ylim([0.8,1.1])
nexttile(1);
l=legendLatex(gcf,{ '$V_4^\mathrm{ref}\pm 1.3d$','$V_4^\mathrm{ref}\pm d$'},p2); 
set(l,'location','southwest')

nexttile(2);
l=legendLatex(gcf,{'$\Delta P_L = {}$ 0.25\,p.u.', '$\Delta P_L = {}$ 0.20\,p.u.'}); 
set(l,'location','northwest')
% Qstart = 0;
% plot(y{1}.TIME,Qstart+pick_out*PAR_db.PODQ.input_func(y{1}.TIME',1),'--','color',[1,1,1]*0.6)


saveLatex(fig1,'sim2_dead_band',fig_save_opt)