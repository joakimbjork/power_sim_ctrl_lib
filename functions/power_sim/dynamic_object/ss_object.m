function [out1, out2, out3, out4] = ss_object(create_object, in1, in2, in3, in4, in5, in6)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-23


if create_object >= 1
    sys = ss(in1); % Given state space system
    if exist('in2','VAR')
        PAR = in2;
    else
        PAR = struct();
    end
    if isfield(PAR,'x0')
        x0 = PAR.x0; % State vector
    else
        x0 = zeros(0,1);
    end
    if create_object == 2
        save_output = false;
    else
        save_output = true;
    end
    if save_output
        if isfield(PAR,'u_count')
            u_count = PAR.u_count; % Number of output signals in the network model  
        else
            u_count = 0;
        end
    end
    if isfield(PAR,'y0')
        y0 = PAR.y0; % Algebraic variable vector
    else
        y0 = zeros(0,1);
    end

       
    G = struct();
    G.type = 1;
    G.sys = sys; % The state space system
    G.nx = size(sys.A,1); % Number of states
    G.idx_nx = length(x0) + (1:G.nx); % Objects position in state vector
    
    [m,m2] = size(sys); % Number of measurements = number of outputs. This can be changed
    assert(m==m2);
    
    if save_output
        u_idx = 1:m;
        G.u_idx = u_count + u_idx;
    end
    
    %% Saturation
    if exist('in3','VAR') && ~isempty(in3) % Output limit
        max_out = in3;
        m_ = size(max_out,1);
        if m_<m
            max_out_add = ones(m-m_,1).*max_out(1,:);
            max_out = [max_out;max_out_add];
        end
        if size(max_out,2)==2
            G.min = max_out(:,1);
            G.max = max_out(:,2);
        else
            G.max = abs(max_out(:,1)); 
            G.min = -G.max(:,1);
        end
        G.saturation = true;
    else
        G.saturation = false;
    end
   
    %% Ramp rate
    if exist('in4','VAR') && ~isempty(in4)
        ramp_lim = in4; % Ramp rate limit
        if size(ramp_lim,2)==2
            G.ramp_min = ramp_lim(:,1);
            G.ramp_max = ramp_lim(:,2);
        else
            G.ramp_max = abs(ramp_lim(:,1)); 
            G.ramp_min = -G.ramp_max(:,1);
        end

        s = tf('s');
        sys2 = eye(m)*ss(0);
        sys3 = eye(m)*ss(0);
        for i = 1:m
            F = minreal(sys(i,i));
            F2 = ss(1/s);
            F3 = ss(1/s);
            if F.D == 0 && isempty(F.B) % No control at this bus
                F = ss(0);
                F2 = ss(0);
                F3 = ss(0);
            elseif F.D == 0 && ~isempty(F.B) % No extra state needed for the derivative block since F is strictly proper
                F = minreal(ss( F*s ));    
            else
                disp('Adding an extra state in ramp_rate_limit in order to make the derivative block proper. Consider making controller strictly proper so that this can be avoided.')
                F = minreal(ss( F*s/(s*1e-6 + 1) ));          
            end
            % if ~isempty(F.B) || ~isempty(F.D) % 
            %     disp('Adding an extra state in ramp_rate_limit in order to make the derivative block proper. Consider making controller strictly proper so that this can be avoided.')
            %     F = minreal(ss( F*s/(s*1e-3 + 1) ));
            % end
            sys(i,i) = F;
            sys2(i,i) = F2;
            sys3(i,i) = F3;
        end
        G.sys = sys; % The state space system with derivative
        G.sys2 = sys2; % Integral to get correct output
        G.sys3 = sys3; % Integral to get nominal output (w.o. ramp rate)
        
        % Anti ramp drift controller
        if exist('in6','VAR') && ~isempty(in6)
            if size(in6,2) == 1 
                sys4 = eye(m)*ss(in6);
            elseif size(in6,2) == m 
                sys4 = ss(in6);
            else
                disp('Anti ramp drift controller has wrong dimension')
            end
        else
%             disp('Anti drift controller for ramp rate not specified. Loading default prop controller')
            sys4 = eye(m)*ss(100);
        end
        
        G.sys4 = sys4; % Anti-drift control to fix steady error due to ramp rate in closed-loop
        G.nx = size(sys.A,1)+size(sys2.A,1)+size(sys3.A,1)+size(sys4.A,1); % Number of states
        G.idx_nx = length(x0) + (1:G.nx); % Objects position in state vector
        G.idx_ramp_error = length(x0) + (G.nx+1-m:G.nx);  
        G.ramp_rate = true;
    else
        G.ramp_rate = false;
    end
    
    %% Dead band
    if exist('in5','VAR') && ~isempty(in5) % Dead band
        dead_band = in5;
        if size(dead_band,2)==2
            G.dead_min = dead_band(:,1);
            G.dead_max = dead_band(:,2);
        else
            G.dead_max = abs(dead_band(:,1)); % Default output limit [p.u.]
            G.dead_min = -G.dead_max(:,1);
        end
        G.dead_band = true;
    else
        G.dead_band = false;
    end    
    
    %% Output of created ss_object
    PAR.x0 = [x0; zeros(G.nx,1)]; % Updated state vector
    PAR.nx = length(PAR.x0); % Update total number of states
    if save_output
        PAR.u_count = u_count + m; % Update total number of outputs
    end
    PAR.y0 = y0;
%     if G.ramp_rate == true
%        PAR.y0 = [PAR.y0; zeros(m,1)]; % New algebraic variables for the ramp drift correction
%        PAR.ny = length(PAR.x0); % Update total number of algrebraic variables
%     end
    
    out1 = G;
    out2 = PAR;    
else
    %% Simulate existing ss_object
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % u = u0 + du = u0 + G*y
    
    G = in1;
    x0 = in2;
    if nargin == 3
        [~,~,C1,~] = ssdata(G.sys); % The state space system with derivative 
        m = size(C1,1); % Number of measurements = number of outputs
        y = zeros(m,1);
        u0 = y;
    else
        y = in3;
        u0 = in4;
    end
    
    x = x0(G.idx_nx); % pick out states of the dynamic object
    
    %% Dead band (input limitation)
    if G.dead_band
        for i = 1:length(y)
            yi = - y(i);
            if yi < G.dead_min(i)
                yi = yi - G.dead_min(i);
            elseif yi > G.dead_max(i)
                yi = yi - G.dead_max(i);
            else
                yi = 0;
            end
            y(i) = - yi;
        end
    end
    
    %% Ramp rate and saturation (output limitation)
    if G.ramp_rate
        [A1,B1,C1,D1] = ssdata(G.sys); % The state space system with derivative 
        m = size(C1,1); % Number of measurements = number of outputs
        [A2,B2,C2,~] = ssdata(G.sys2); % Integral to get correct output
        [A3,B3,C3,~] = ssdata(G.sys3); % Integral to get nominal output (w.o. ramp rate)
        [A4,B4,C4,D4] = ssdata(G.sys4); % Anti-drift control to fix steady error due to ramp rate in closed-loop
        % OBS! D2, D3 = 0;        

%       Ramp rate error
               
        n1 = size(A1,1);
        n2 = size(A2,1);
        n3 = size(A3,1);
        n4 = size(A4,1);
        
        x1 = x(1:n1);
        x2 = x(n1+(1:n2));
        x3 = x(n1+n2+(1:n3));
        x4 = x(n1+n2+n3+(1:n4)); 
                     
        % The state space system with derivative
        f1_xy = A1*x1 + B1*y;
        du1 = C1*x1 + D1*y;
        
        % Integral to get nominal output (w.o. ramp rate)
        f3_xy = A3*x3 + B3*du1;
        du = C3*x3; % Nominal output 

        % Ramp rate limited output
        du_ramp_lim = C2*x2; 
        
        %% Saturation with ramp rate limiter
        du = du + u0; % Nominal ouput
        du_sat = du;
        if G.saturation
            for i = 1:length(du_sat)
                du_sat(i) = min( max(G.min(i),du_sat(i)), G.max(i));
            end
        end 

        u = du_ramp_lim + u0; % Saturation and ramp rate limited output
        if G.saturation
            for i = 1:length(u)
                u(i) = min( max(G.min(i),u(i)), G.max(i));
            end
        end            
        
        % Accumulated ramp rate error
        acc_error = du_sat - u;
        
        % Anti-drift control to fix steady error due to ramp rate
        f4_xy = A4*x4 + B4*acc_error;
        ramp_cor = C4*x4 + D4*acc_error;   
        du1_cor = du1 + ramp_cor;                  
          
        %% Ramp rate limiter
        for i = 1:m
            du1_cor(i) = min( max(G.ramp_min(i),du1_cor(i)), G.ramp_max(i));
        end
        
        % Integral to get correct ramp rate limited output
        f2_xy = A2*x2 + B2*du1_cor ;
        
        f_xy = [f1_xy; f2_xy; f3_xy; f4_xy];
    
    else % No ramp rate limit
        [A,B,C,D] = ssdata(G.sys);
        if G.nx > 0
            f_xy = A*x + B*y;
            du = C*x + D*y;
        else
            du = D*y; % Direct term only
            f_xy = [];
        end
        %% Saturation without ramp rate limiter
        du = du + u0;
        u = du;
        if G.saturation
            for i = 1:length(u)
                u(i) = min( max(G.min(i),u(i)), G.max(i));
            end
        end            
    end
     
    
    %% Output
    out1 = f_xy; % Derivatives
    out2 = u; % Saturation limited output
    out3 = du;
    if G.ramp_rate
        out4 = acc_error;
    else
        out4 = 0;
    end
end

