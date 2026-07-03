function [gain, theta, tlim, ty, tu] = findGainPhase(t,y,u,omega)
% Skapad 2022-02-11
% Joakim Björk, joakim.bjork@svk.se

f = omega/(2*pi); % rad/s -> Hz
[max_y, idx_y] = findPeaks(t,y,f); % Kommer behöva fixas för att ta hand om saturation
[max_u, idx_u] = findPeaks(t,u,f);

dt = t(2)-t(1);
time_shift = dt * (idx_u-idx_y);
phase_shift = 2*pi*time_shift*f; 
window = 5; % Kolla över fler perioder för att få ett bättre värde på förstärkningen

gain = max(max_y(end-window:end)/max(max_u(end-window:end))); % Förstärkning
theta = phase_shift(end); % Fasförskjutning [rad]

% Tidsfönster för beräkning av fasförkjutningen
ty = idx_y(end)*dt;
tu = idx_u(end)*dt;

tlim = [t(end)-window/f, t(end)];
end

