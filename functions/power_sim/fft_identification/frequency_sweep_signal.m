function u = frequency_sweep_signal(T,amp,period_times,periods_per_level)
%UNTITLED Summary of this function goes here
%Detailed explanation goes here
N = length(period_times);
time_per_level = period_times*periods_per_level;
A = ones(N);
A = triu(A);
time_at_end_of_level = time_per_level*A;
idx = find(time_at_end_of_level>=T);
if isempty(idx)
    idx = 1;
else
    idx = idx(1);
end
period_time = period_times(idx);
if idx >= 2
    T = T-time_at_end_of_level(idx-1);
end
u_freq = 1/period_time;
u = amp*cos(2*pi*u_freq*T);
% u = amp*cos(2*pi*u_freq*T).*min(1,T*u_freq/2);
end

