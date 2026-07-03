function [modes1, modes2] = split_mode_object(modes,idx1)
% Split modes into modes1 and modes2 based on idx1
idx2 = 1:modes.N;
idx2(idx1) =[];
modes1 = pick_out_modes(modes,idx1);
modes2 = pick_out_modes(modes,idx2);
end

