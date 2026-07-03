function [PAR,opt] = load_PODQ(PAR,opt)


%%
Sb = PAR.Sb;
nbus = PAR.nbus;

if isfield(opt,'PODQ')
    if isfield(opt, 'PODQ_max_out')
        max_out = opt.PODQ_max_out;
    else
        max_out = [];
    end

    if isfield(opt, 'PODQ_ramp_rate')
        ramp_rate = opt.PODQ_ramp_rate;
    else
        ramp_rate = [];
    end

    if isfield(opt, 'PODQ_dead_band')
        dead_band = opt.PODQ_dead_band;
    else
        dead_band = [];
    end


    if ~isfield(opt, 'PODQ_type')
        opt.PODQ_type = 1;
    end




    if opt.PODQ_type == 1 || opt.PODQ_type == 2
        [G, PAR] = ss_object(1, opt.PODQ/Sb, PAR,  max_out/Sb, ramp_rate/Sb, dead_band);
        PAR.PODQ = G;
        if opt.PODQ_type == 1
            disp('Loading linear PODQ: VOLT [p.u.] -> Q [Mvar]')
        elseif opt.PODQ_type == 2
            disp('Loading linear PODQ: ANG [radian] -> Q [Mvar]')
            PAR.PODQ.type = 2;
        end



    elseif opt.PODQ_type == 3 || opt.PODQ_type == 4
        disp('Loading inverter model for PODQ: Q [Mvar] -> Q [Mvar]')
        [G, PAR] = ss_object(1, opt.PODQ, PAR,  max_out/Sb, ramp_rate/Sb, dead_band);
        PAR.PODQ = G;
        PAR.PODQ.type = opt.PODQ_type;

        %% Default controller 1 (e.g. derivative controller with small saturation)
        disp('Loading voltage controller 1: VOLT [p.u.] -> Q [Mvar]')

        if isfield(opt, 'PODQ1_max_out')
            max_out = opt.PODQ1_max_out;
        else
            max_out = [];
        end

        if isfield(opt, 'PODQ1_ramp_rate')
            ramp_rate = opt.PODQ1_ramp_rate;
        else
            ramp_rate = [];
        end

        if isfield(opt, 'PODQ1_dead_band')
            dead_band = opt.PODQ1_dead_band;
        else
            dead_band = [];
        end

        [G, PAR] = ss_object(1, opt.PODQ1/Sb, PAR,  max_out/Sb, ramp_rate/Sb, dead_band);
        PAR.PODQ.K1 = G;

        %% Default controller 2 (e.g. proportional controller with max saturation limit)
        disp('Loading voltage controller 2: VOLT [p.u.] -> Q [Mvar]')

        if isfield(opt, 'PODQ2_max_out')
            max_out = opt.PODQ2_max_out;
        else
            max_out = [];
        end

        if isfield(opt, 'PODQ2_ramp_rate')
            ramp_rate = opt.PODQ2_ramp_rate;
        else
            ramp_rate = [];
        end

        if isfield(opt, 'PODQ2_dead_band')
            dead_band = opt.PODQ2_dead_band;
        else
            dead_band = [];
        end

        [G, PAR] = ss_object(1, opt.PODQ2/Sb, PAR,  max_out/Sb, ramp_rate/Sb, dead_band);
        PAR.PODQ.K2 = G;

        %% Communication delay from K1 and K2
        if opt.PODQ_type == 4
            disp('Loading communication between K1, K2 and the converter')

            if isfield(opt, 'COM_max_out')
                max_out = opt.COM_max_out;
            else
                max_out = [];
            end

            if isfield(opt, 'COM_ramp_rate')
                ramp_rate = opt.COM_ramp_rate;
            else
                ramp_rate = [];
            end

            if isfield(opt, 'COM_dead_band')
                dead_band = opt.COM_dead_band;
            else
                dead_band = [];
            end

            [G, PAR] = ss_object(1, opt.PODQ_COM, PAR,  max_out/Sb, ramp_rate/Sb, dead_band);
            PAR.PODQ.COM = G;


        end

        %% Aux controller (e.g. strong proportional controller with dead band activation)
        disp('Loading aux voltage controller: VOLT [p.u.] -> Q [Mvar]')

        if isfield(opt, 'PODQaux_max_out')
            max_out = opt.PODQaux_max_out;
        else
            max_out = [];
        end

        if isfield(opt, 'PODQaux_ramp_rate')
            ramp_rate = opt.PODQaux_ramp_rate;
        else
            ramp_rate = [];
        end

        if isfield(opt, 'PODQaux_dead_band')
            dead_band = opt.PODQaux_dead_band;
            if opt.PODQ_type == 3 % Check for absolute dead band
                idx = diag(dcgain(opt.PODQaux))~=0;
                if any(dead_band(idx,1)-PAR.U0(idx)>0)
                    warning('Simulation will not start at steady state')
                    disp('One of the PODQ regulators is initiated below the dead band')
                elseif any(dead_band(idx,2)-PAR.U0(idx)<0)
                    warning('Simulation will not start at steady state')
                    disp('One of the PODQ regulators is initiated above the dead band')
                end
            end
        else
            dead_band = [];
        end

        [G, PAR] = ss_object(1, opt.PODQaux/Sb, PAR,  max_out/Sb, ramp_rate/Sb, dead_band);
        PAR.PODQ.Kaux  = G;

    elseif opt.PODQ_type == 5
        disp('Loading inverter model for PODQ: Q [Mvar] -> Q [Mvar]')
        [G, PAR] = ss_object(1, opt.PODQ, PAR,  max_out/Sb, ramp_rate/Sb, dead_band);
        PAR.PODQ = G;
        PAR.PODQ.type = opt.PODQ_type;
        %% Inner controller (e.g. PODQ)
        disp('Loading voltage controller 1: VOLT [p.u.] -> Q [Mvar]')

        if isfield(opt, 'PODQ1_max_out')
            max_out = opt.PODQ1_max_out;
        else
            max_out = [];
        end

        if isfield(opt, 'PODQ1_ramp_rate')
            ramp_rate = opt.PODQ1_ramp_rate;
        else
            ramp_rate = [];
        end

        if isfield(opt, 'PODQ1_dead_band')
            dead_band = opt.PODQ1_dead_band;
        else
            dead_band = [];
        end

        [G, PAR] = ss_object(1, opt.PODQ1/Sb, PAR,  max_out/Sb, ramp_rate/Sb, dead_band);
        PAR.PODQ.K1 = G;

        %% Outer controller (e.g. proportional voltage control)
        disp('Loading voltage controller 2: VOLT [p.u.] -> Q [Mvar]')

        if isfield(opt, 'PODQ2_max_out')
            max_out = opt.PODQ2_max_out;
        else
            max_out = [];
        end

        if isfield(opt, 'PODQ2_ramp_rate')
            ramp_rate = opt.PODQ2_ramp_rate;
        else
            ramp_rate = [];
        end

        if isfield(opt, 'PODQ2_dead_band')
            dead_band = opt.PODQ2_dead_band;
        else
            dead_band = [];
        end

        [G, PAR] = ss_object(1, opt.PODQ2/Sb, PAR,  max_out/Sb, ramp_rate/Sb, dead_band);
        PAR.PODQ.K2 = G;

        %% Q controller (e.g. PI controller)
        disp('Loading outer Q controller: Q [Mvar] -> Q [Mvar]')


        [G, PAR] = ss_object(1, opt.PODQ_KQ, PAR);
        PAR.PODQ.KQ = G;


    end



    if isfield(opt, 'PODQ_CL') % Should test be perfromed in closed loop
        PAR.PODQ.CL = opt.PODQ_CL;
    else
        PAR.PODQ.CL = 1; % Loop is closed by default
    end

end