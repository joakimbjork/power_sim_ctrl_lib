opt_CIGRE2024;

simopt = struct();
simopt.show_active_power_flow = true;
simopt_load_trip = simopt;
simopt_load_trip.dPL = -[0.25;-0.25;0;0]; % Fixed size disturbance at bus 1 and 2


simopt_reactive_step = simopt;
simopt_reactive_step.dQL = zeros(4,1);
simopt_reactive_step.dQL(3) = 0; % Disturbance in p.u.
simopt_dist = simopt_reactive_step;


sim_i = 1; % Short simulation <------- Simulates voltage reference step
% sim_i = 2; % Long simulation <------- Simulates load trip

if sim_i == 1
    Tend = 10;
    IBR = pade(H,20);
    dVref = 0.02*1;
    simopt_dist = simopt_reactive_step;
    Htd = pade(Hcom,20);
else
    Tend = 50;
    IBR = pade(H,20);
    dVref = 0.02*0;
    simopt_dist = simopt_load_trip;
    Htd = pade(Hcom,5);
end

step_func = @(T,u) dVref*ones(4,1).*(T>1); % Step function

u_max = 1/3;
opt.PODQ_max_out = [-1,1].*ones(PAR.nbus,1)*u_max*PAR.Sb;
% PODQ
opt_voltage = opt;
opt_voltage.PODQ = tf(zeros(4));
opt_voltage.PODQ(bus_idx,bus_idx) = IBR*K*PAR.Sb; % The controller in MVAR/p.u. voltage
opt_voltage.PODQ_CL = 1;
PAR_PODQ = data_PODQ_test_system(opt_voltage);
PAR_PODQ.PODQ.input_func = step_func; % Step function


% Voltage control
opt_voltage = opt_default;
u_max = 1/3;
opt_voltage.PODQ2(bus_idx,bus_idx) = Kc*PAR.Sb; % The controller in MVAR/p.u. voltage
opt_voltage.PODQ2_max_out = [-1,1].*ones(PAR.nbus,1)*u_max*PAR.Sb;
opt_voltage.PODQ_COM(bus_idx,bus_idx) = Htd; % Communication delay
PAR_voltage = data_PODQ_test_system(opt_voltage);
PAR_voltage.PODQ.input_func = step_func; % Step function

% Pure damping control
opt_voltage = opt;
opt_voltage.PODQ = tf(zeros(4));
opt_voltage.PODQ(bus_idx,bus_idx) = IBR*K_POD*PAR.Sb; % The controller in MVAR/p.u. voltage
opt_voltage.PODQ_CL = 1;
PAR_damping = data_PODQ_test_system(opt_voltage);
PAR_damping.PODQ.input_func = step_func; % Step function

% No control
opt_voltage = opt;
opt_voltage.PODQ = tf(zeros(4));
% opt_voltage.PODQ(bus_idx,bus_idx) = IBR*F_V*PAR.Sb; % The controller in MVAR/p.u. voltage
opt_voltage.PODQ_CL = 1;
PAR_no_ctrl = data_PODQ_test_system(opt_voltage);
PAR_no_ctrl.PODQ.input_func = step_func; % Step function


y = cell(0);
[y{1}] = runsim(PAR_voltage,Tend,simopt_dist);
[y{2}] = runsim(PAR_PODQ,Tend,simopt_dist);
% [y{3}] = runsim(PAR_damping,Tend,simopt_dist);
if sim_i == 2
    [y{3}] = runsim(PAR_no_ctrl,Tend,simopt_dist);
elseif sim_i == 1   
    [y{3}] = runsim(PAR_damping,Tend,simopt_dist);
%     [y{4}] = runsim(PAR_no_ctrl,Tend,simopt_dist);
end


[fig1,~,co] = figureLatex(4,3.5);
tile1 = tiledLatex(2,1);
% bus_idx = 4;
% title(tile1,['Voltage and reactive power at bus ', num2str(bus_idx)])
T = Tend(end);
[fig2,~,co] = figureLatex(4,3.5);
tile2 = tiledLatex(2,1);
% title(tile2,['Rotor angle and power flow'])

for idx = 1:length(y)
    if sim_i == 1
        if idx == 3
            style = '--';
        else
            style = '-';
        end
        col = co(idx,:);
        idx_ = idx;
    elseif sim_i == 2
        style = '-';
        if idx == 1
            col = [1,1,1]*0.6;
            idx_ = 3;
        else
            col = co(idx-1,:);
            idx_ = idx-1;
        end
        
    end
    y_ = y{idx_};

    % downsample
    n_sample = 25;
    t_length = length(y_.TIME);
    t_idx = false(t_length,1);
%     t_idx(1:n_sample:t_length)=true;
%     t_idx(end) = true;
%     t_idx(y_.TIME==1) = true;

    if sim_i == 1
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
        y_.TIME(y_.TIME==1) = 1+TD
%         t_y = t_idx;
%         t_y(2) = t_y(1);
    else
        if idx_ == 3
            t_idx(1:20:t_length)=true;
        else
            t_idx(1:n_sample:t_length)=true;
        end
              
        t_idx(end) = true;
        t_idx(y_.TIME==1) = true;
%         t_y = t_idx;
    end

    figure(fig1)
    nexttile(1)
    plot(y_.TIME(t_idx),y_.VOLT(t_idx,bus_idx),style,'color',col); hold all
    ylabel('Voltage, $V_4$ [p.u.]')
    xlim([0,T])
    
    nexttile(2)
    if sim_i == 2 && idx_ == 3
        plot(y_.TIME([1,end]),-y_.u_QL([1,end],bus_idx),style,'color',col); hold all
    else
        plot(y_.TIME(t_idx),-y_.u_QL(t_idx,bus_idx),style,'color',col); hold all
    end
    
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
%%

if sim_i == 1
    % close(fig2)
    figure(fig1)
    nexttile(1)

    xticklabels([])

    t_begin_avg = 6;
    [~,idx_begin_avg] = min(abs(y{1}.TIME-t_begin_avg))
    Vstart = PAR_voltage.U0(4);
    Vend_ = y{1}.VOLT(idx_begin_avg:end,4);
    Vend = mean(Vend_);
    Qend_ = -y{1}.u_QL(idx_begin_avg:end,4) ;
    Qend = mean(Qend_)
    dVsim = Vstart-Vend
    droop = dVsim/(Qend/1000)
    
    
    step_func = @(T,u) dVref*(T>1); % Step function
%     plot(y{1}.TIME,Vstart+step_func(y{1}.TIME,1),'--','color',[1,1,1]*0.6)
    plot([0,1,1,Tend],Vstart+[0,0,dVref,dVref],'--','color',[1,1,1]*0.6)

    l=legendLatex(gcf,{'Voltage control',...
                       'POD-Q',...
                       'Pure POD-Q',...
                       '$V_4^\mathrm{ref}$'});  
    set(l,'location','southeast')
    % saveas(fig1,[file_path,'ref_step_PODQ.eps'],'epsc')
else
    figure(fig1)
%     x_ticks = [0,10,20,30,40,50,60,70,80];
    nexttile(1)
%     set(gca,'XTick',x_ticks);
    xticklabels([])
    nexttile(2)
    l=legendLatex(gcf,{'No control',...
                       'Voltage control',...
                       'POD-Q'});  
    set(l,'location','east')
%     ylim([0.8,1.1])
    nexttile(2)
%     ylim([-0.1,1.1])
    
    figure(fig2)
    nexttile(1)
%     set(gca,'XTick',x_ticks);
    xticklabels([])
    nexttile(2) 
    l=legendLatex(gcf,{'No control',...
                       'Voltage control',...
                       'POD-Q'});  
    set(l,'location','east')

%     ylim([-20,150])
    nexttile(2)
%     ylim([-0.2,1.5])
    saveLatex(fig1,'sim1_PODQ_reactive',fig_save_opt)
    saveLatex(fig2,'sim1_PODQ_active',fig_save_opt)
end