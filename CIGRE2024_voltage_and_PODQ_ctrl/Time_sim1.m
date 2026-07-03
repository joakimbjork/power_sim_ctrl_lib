opt_CIGRE2024;
simopt = struct();
simopt.show_active_power_flow = true;
simopt_load_trip = simopt;
simopt_load_trip.dPL = 1*[0.5;-0.5;0;0]; % Fixed size disturbance at bus 1 and 2

simopt_reactive_step = simopt;
simopt_reactive_step.dQL = zeros(4,1);
simopt_reactive_step.dQL(3) = 0; % Disturbance in p.u.
simopt_dist = simopt_reactive_step;

Tend = 10;


dVref = 0.02*1;
step_func = @(T,u) dVref*ones(4,1).*(T>1); % Step function


% IBR with out time delay
opt_voltage = opt_default;
opt_voltage.PODQ2(bus_idx,bus_idx) = Kc*PAR.Sb; % The controller in MVAR/p.u. voltage
PAR_voltage = data_PODQ_test_system(opt_voltage);
PAR_voltage.PODQ.input_func = step_func; % Step function

% IBR with time delay
opt_voltage.PODQ_COM(bus_idx,bus_idx) = pade(Hcom,20); % Communication delay
PAR_voltage_td = data_PODQ_test_system(opt_voltage);
PAR_voltage_td.PODQ.input_func = step_func; % Step function

y = cell(0);
[y{1}] = runsim(PAR_voltage_td,Tend,simopt_dist);
[y{2}] = runsim(PAR_voltage,Tend,simopt_dist);


[fig1,~,co] = figureLatex(4,3.5);
tile1 = tiledLatex(2,1);
% bus_idx = 4;
% title(tile1,['Voltage and reactive power at bus ', num2str(bus_idx)])
T = Tend(end);
[fig2,~,co] = figureLatex(4,3.5);
tile2 = tiledLatex(2,1);

for idx = 1:length(y)
    y_ = y{idx};

    % downsample
    n_sample = 5;
    if idx == 1
        n_sample = 30;
    end
    t_length = length(y_.TIME);
    t_idx = false(t_length,1);

    if idx == 1
        i_first = 1;
        if 0 % Manual filtering of the time delay simulation (to save memory
            % space). This removes the pade aproximation flickering
            i_first = find(y_.TIME>1+TD);
            i_first = i_first(1)-1;
        end
        t_idx(i_first:n_sample:t_length)=true;
        t_idx(end) = true;
        t_idx(1) = true;
        t_idx(y_.TIME==1) = true;
        y_.TIME(y_.TIME==1) = 1+TD;
        t_y = t_idx;
        %         t_y(2) = t_y(1);
    else
        t_idx(1:n_sample:t_length)=true;
        t_idx(end) = true;
        t_idx(y_.TIME==1) = true;
        t_y = t_idx;
    end



    figure(fig1)
    nexttile(1)
    plot(y_.TIME(t_idx),y_.VOLT(t_y,bus_idx)); hold all
    ylabel('Voltage [p.u.]')
    xlim([0,T])

    nexttile(2)
    plot(y_.TIME(t_idx),-y_.u_QL(t_y,bus_idx) ); hold all
    ylabel('Reactive power [p.u.]')
    xlim([0,T])
    xlabel('Time [s]')

    figure(fig2)
    nexttile(1)
    plot(y_.TIME(t_idx),y_.DELTA(t_y,1)-y_.DELTA(t_y,2)); hold all
    ylabel('Rotor angle, $\delta_1-\delta_2$ [$^\circ$]')
    xlim([0,T])

    nexttile(2)
    from = 1; to = 2;
    plot(y_.TIME(t_idx),y_.Pline(t_y, PAR.ng+PAR.nbus*(to-1)+from )); hold all
    xlim([0,T])
    ylabel(['Active power, bus ',num2str(from),'$\rightarrow$', num2str(to), ' [p.u.]'])
    xlabel('Time [s]')
end
% close(fig2)
figure(fig1)
nexttile(1)

t_begin_avg = 6;
[~,idx_begin_avg] = min(abs(y{1}.TIME-t_begin_avg))
Vstart = PAR_voltage.U0(4);
Vend_ = y{1}.VOLT(idx_begin_avg:end,4);
Vend = mean(Vend_);
Qend_ = -y{1}.u_QL(idx_begin_avg:end,4) ;
Qend = mean(Qend_)
dVsim = Vstart-Vend
droop = dVsim/(Qend/1000)

% step_func = @(T,u) dVref*(T>1); % Step function
plot([0,1,1,Tend],Vstart+[0,0,dVref,dVref],'--','color',[1,1,1]*0.6)
nexttile(2)
% plot([y{1}.TIME(1),y{1}.TIME(end)],K0*[dVref,dVref]*PAR.Sb,'--','color',[1,1,1]*0.6)
plot([y{1}.TIME(1),y{1}.TIME(end)],1*[Qend,Qend],'-','color',[1,1,1]*0.6)
plot([y{1}.TIME(1),y{1}.TIME(end)],0.9*[Qend,Qend],'--','color',[1,1,1]*0.6)
xline(1)
xline(1+t_1)
% plot(y{1}.TIME((end-1500):end),Vend_,'color',[1,0,0]*0.6)
nexttile(1)
l=legendLatex(gcf,{'$V_4$, $\tau={}$75\,ms','$V_4$, $\tau={}$0\,ms','$V_4^\mathrm{ref}$'});
set(l,'location','southeast')
nexttile(2)
l=legendLatex(gcf,{'$Q_4$, $\tau={}$75\,ms','$Q_4$, $\tau={}$0\,ms','100\% of $\Delta Q_4$','90\% of $\Delta Q_4$'});
set(l,'location','southeast')

nexttile(2)
% xaxisproperties= get(gca, 'XAxis');
% xaxisproperties.TickLabelInterpreter = 'latex'; % latex for x-axis
x_axis_ticks = [0,1,1+t_1,5,10,15,20,25];
xticks(x_axis_ticks)
xticklabels({0,'','',5,10,15,20,25})
h = text([1,1+t_1], [0,0]-0.0015,...%-abs(mean(ylims))/2,...
    {'$t_0$','$t_0+t_1$'},'HorizontalAlignment','center','VerticalAlignment','cap','color', [.15 .15 .15],'fontsize',10);
nexttile(1)
xticks(x_axis_ticks)
xticklabels([])

saveLatex(fig1,'ref_step_Voltage',fig_save_opt)