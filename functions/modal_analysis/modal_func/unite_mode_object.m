function [m1] = unite_mode_object(m1,m2)
m1.e = [m1.e; m2.e];
m1.v = [m1.v, m2.v];
m1.f = [m1.f; m2.f];
m1.d = [m1.d; m2.d];
m1.observed = [m1.observed; m2.observed];
m1.N = length(m1.e);
end

