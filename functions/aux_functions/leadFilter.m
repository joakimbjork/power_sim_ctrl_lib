function F = leadFilter(phase_add,wc)
% Skapad 2022-03-16
% Joakim Björk, joakim.bjork@svk.se

% adds phase_add [rad] at frequency wc [rad/s].
if phase_add == 0
    F = tf(1);
else
    x = tan(abs(phase_add));
    p = -2-4*x^2;
    q = 1;
    beta = -p/2 - sqrt(p^2/4 -q);
    tD = 1/(wc*sqrt(beta));
    
    s= tf('s');
    F = (tD*s+1)/(beta*tD*s+1);
    
    if phase_add<0
        F = inv(F);
    end
end
end
