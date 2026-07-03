function [new] = pick_out_modes(modes,idx_modes)
new = struct();
new.e = modes.e(idx_modes);
new.v = modes.v(:,idx_modes);
new.f = modes.f(idx_modes);
new.d = modes.d(idx_modes);
new.observed = modes.observed(idx_modes);
new.N = length(idx_modes);
end

