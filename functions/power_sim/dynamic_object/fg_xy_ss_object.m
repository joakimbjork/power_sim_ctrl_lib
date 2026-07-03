function [fg, u] = fg_xy_ss_object(T,xy0,PAR)
%FG_XY_SS_OBJECT Summary of this function goes here
%   Detailed explanation goes here
% [out1, out2, out3, out4] = ss_object(0, in1, in2, in3, in4, in5, in6)

f_xy = zeros(PAR.nx,1);
x0 = xy0(1:PAR.nx);

%% Simulate existing ss_object
    % u = u0 + du = u0 + G*y
    
%     G = in1;
%     x0 = in2;
%     y = in3;
%     u0 = in4;

dy = PAR.dy;
u0 = PAR.u0;

if isfield(PAR,'G')
    if PAR.G.type == 1 % Standard ss object
        if isfield(PAR.G,'input_func')
            dy = PAR.G.input_func(T,dy);
        end
        [f_xy(PAR.G.idx_nx), u, u_ref] = ss_object(0, PAR.G, x0, dy, u0);
    end
    fg = f_xy;
    u = [u;u_ref];
end



% g_xy = [];
% fg=([f_xy; g_xy]);


end

