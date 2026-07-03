function [PAR,opt] = load_CONV(PAR,opt)
% Joakim Björk, joakim.bjork@svk.se, 2024-09-10

% Converter object.

%%
Sb = PAR.Sb;

if isfield(opt,'CONV')
    obj = opt.CONV;
    Cbus = obj.bus;
    m = length(Cbus);
    s=tf('s');
    
    %% Base parameters
    if ~isfield(obj, 'MBASE') % Size of inverter in pu power base. Ex: 100 MW inverter -> MBASE = 100/Sb;
        obj.MBASE = ones(m,1);
    end

    if ~isfield(obj, 'Imax') % Size of inverter in pu power base. Ex: 100 MW inverter -> MBASE = 100/Sb;
        obj.Imax = ones(m,1).*obj.MBASE;
        obj.Imax_inner = obj.Imax*1.05; % Used by GFM. Need extra capacity in internal current control for the GFM syncronization to work
    end

    if ~isfield(obj, 'X')
        obj.X = ones(m,1)*0.05; % Impedance in per unit
    end

    obj.X = obj.X./obj.MBASE;
    % obj.X = obj.X./1000;

    if isfield(opt, 'data') && isfield(opt.data, 'SCP') % Has SCP been calculated at network buses?
        SCP = opt.data.SCP(Cbus);
        ESCP = 1./(1./SCP+obj.X);
        opt.data.ESCP = ESCP; % Extended short circuit power (admittance in pu)
    else 
        ESCP = ones(m,1);
    end


    %% Measurement blocks for external control functions

    if ~isfield(obj, 'P_measurement')
        K = 100/(s+100); % Measurement
        obj.P_meas = minreal(eye(m)*K);
    end

    if ~isfield(obj, 'Q_measurement')
        K = 100/(s+100); % Measurement
        obj.Q_meas = minreal(eye(m)*K);
    end

    if ~isfield(obj, 'freq_measurement')
        K = 100/(s+100); % Measurement
        obj.freq_meas = minreal(eye(m)*K);
    end


    %% Syncronizing control

    if isfield(obj,'GFM')
        disp('Loading grid forming converter')
        obj.type = 3;
        if obj.GFM >= 1
            s = tf('s');
            K_sync = tf(zeros(m));
            for i = 1:m    
                if obj.GFM == 2
                    wi = 10;
                    ki_sync = wi^2/ESCP(i);
                    kp_sync = 1*wi/ESCP(i);
                else
                    wi = 150;
                    ki_sync = wi^2/ESCP(i)/10;
                    kp_sync = wi/ESCP(i);
                end
                
                K_sync(i,i) = ki_sync*1/s+kp_sync; % (1+s)/s
                % obj.M(i) = 1/ki;
                obj.ki_sync(i) = ki_sync;
                obj.kp_sync(i) = kp_sync;
            end
            obj.sync_ctrl = minreal(K_sync);
            obj.phase_ctrl = minreal(eye(m)/s);
        else
            obj.sync_ctrl = obj.GFM; % Manual entry
            obj.phase_ctrl = obj.CONV_phase_ctrl; % Manual entry
        end
    elseif isfield(obj,'GFL')
        disp('Loading grid following converter')
        obj.type = 2;
        if obj.GFL == 1
            K_sync = 150*(1/s+1);
            obj.sync_ctrl = minreal(eye(m)*K_sync);
            obj.phase_ctrl = minreal(eye(m)/s); 
        else
            obj.sync_ctrl = obj.GFL; % Manual entry
            obj.phase_ctrl = obj.CONV_phase_ctrl; % Manual entry
        end
    else
        disp('Loading converter with perfect phase alignment')
        obj.type = 1; % Ideal GFL where ANGC = ANG
    end

    %% Current control

    if obj.type == 3 % Grid forming
        % if ~isfield(obj, 'Ed_ctrl') && obj.type == 3 %% Grid forming already has power control
        %     K = 1000/(s+1000); % Measurement
        %     obj.Ed_ctrl = minreal(eye(m)*K);
        % 
        % end
        if ~isfield(obj, 'Id_ctrl')  %% Grid forming already has power control
            Tcc = 1e-3;
            K = 1/(s*Tcc+1); % Measurement
            obj.Id_ctrl = minreal(eye(m)*K);
        end
        if ~isfield(obj, 'Iq_ctrl')  %% Grid forming already has power control
            Tcc = 1e-3;
            K = 1/(s*Tcc+1); % Measurement
            obj.Iq_ctrl = minreal(eye(m)*K);
        end
    
    else    % Grid following or ideal converter
        if ~isfield(obj, 'P_ctrl')  %% Grid forming already has power control
            % K = (100/s + 10);
            % obj.P_ctrl = minreal(eye(m)*K);
            s = tf('s');
            P_ctrl = tf(zeros(m));
            for i = 1:m  
                ki_sync = 1e3;
                kp = 1;
                P_ctrl(i,i) = (ki_sync/s+kp)/ESCP(i); % (1+s)/s
            end
            obj.P_ctrl = P_ctrl;
        end
        if ~isfield(obj, 'Id_ctrl')  %% Grid forming already has power control
        % K = 100/(s+100); % Measurement
        % obj.Id_ctrl = minreal(eye(m)*K);
            Tcc = 1e-3;%1/25;
            K = 1/(s*Tcc+1); % Measurement
            obj.Id_ctrl = minreal(eye(m)*K);
        end
        if ~isfield(obj, 'Iq_ctrl')  %% Grid forming already has power control
            % K = 100/(s+100); % Measurement
            % obj.Iq_ctrl = minreal(eye(m)*K);
            Tcc = 1e-3;%1/25;
            K = 1/(s*Tcc+1); % Measurement
            obj.Iq_ctrl = minreal(eye(m)*K);
        end
    end
    
    
    %% Reactive power control
    if ~isfield(obj, 'Q_ctrl')
        % K = 100*(100/s + 10) ;       
        % obj.Q_ctrl = minreal(eye(m)*K);
        s = tf('s');
        Q_ctrl = tf(zeros(m));
        for i = 1:m  
            ki_sync = 1e3;
            kp = 1;
            Q_ctrl(i,i) = (ki_sync/s+kp)/ESCP(i); % (1+s)/s
        end
        obj.Q_ctrl = Q_ctrl;
    end

   

    if ~isfield(obj, 'P_max_out')
        % obj.P_max_out = zeros(length(cbus),2); %
        % obj.P_max_out(:,1) = -PAR.Sb;
        % obj.P_max_out(:,2) = PAR.Sb;
        obj.P_max_out = [];% pu;
    end

    if ~isfield(obj, 'Q_max_out')
        % obj.Q_max_out = zeros(length(cbus),2); %
        % obj.Q_max_out(:,1) = -PAR.Sb;
        % obj.Q_max_out(:,2) = PAR.Sb;
        obj.Q_max_out = [];% pu;
    end


    [G, PAR] = CONV_object(1, obj, PAR);

    PAR.CONV = G;


    if isfield(obj, 'Storage')
        disp('Loading reacharging storage object with recharing controller specified by obj.Storage')
        [G, PAR] = ss_object(1, eye(m)/s, PAR); % The storage object is modelled as an integrator
        PAR.CONV.Storage = G;
        [G, PAR] = ss_object(1, obj.Storage, PAR); % Recharging controller
        PAR.CONV.Storage.Charge_ctrl = G;
    elseif isfield(obj, 'HVDC_wind')
        disp('Loading HVDC wind turbine system specified by obj.HVDC_wind')
        [PAR,opt] = load_HVDC_wind(PAR,opt);
    end




    % if isfield(obj, 'Q_PF_ctrl')
    %     if obj.Q_PF_ctrl ~= 0
    %         disp('Loading Converter Power Factor Controller')
    %         s = tf('s');
    %         K = 1/(s*0.05+1); % Default power factor controller
    %         max_out = [-1,1]*100;
    %         [G, PAR] = ss_object(1, diag(m)*K, PAR,  max_out);
    %
    %         PC = PAR.P0(Cbus);
    %         QC = PAR.Q0(Cbus);
    %         G.PF = (QC./PC);
    %         PAR.CONV.Q_PF_ctrl = G;
    %     end
    % end
end