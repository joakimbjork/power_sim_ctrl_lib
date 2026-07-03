function [f1,gain1,phase1] = fft_of_frequency_sweep(t,u,period_times,periods_per_level,skip_periods)
% skip_periods = 4;
d_samples = 100; % Samples per period

%% Discretize the simulated result 
N = length(period_times); % Number of frequencies simulated
time_per_level = period_times*periods_per_level;
A = ones(N); A = triu(A);
time_at_end_of_level = time_per_level*A;
A = A - eye(N);
time_at_start_of_level = time_per_level*A;

Tq = [];
for i = 1:N
    Td = period_times(i)/d_samples;
    tq = time_at_start_of_level(i):Td:time_at_end_of_level(i)-Td;
    Tq = [Tq,tq];
end
uq = interp1(t,u,Tq);

%% FFT
samples_per_level = d_samples*periods_per_level;
idx_1 = ((0:(N-1))*samples_per_level)+1;
idx_1 = idx_1 + skip_periods*d_samples; % Skip transient
idx_2 = ((1:N)*samples_per_level);

periods = periods_per_level-skip_periods;
f1= zeros(N,1); % fundamental freq
gain1 = zeros(N,1); % Fundamental gain;
phase1 = zeros(N,1);

for i = 1:N
    Fs = d_samples/period_times(i);
    yq = uq(idx_1(i):idx_2(i));
    L = length(yq);
    yq_fft = fft(yq,L);
    P2 = yq_fft/L;
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(L/2))/L;
    f1(i)= f(periods+1);
    gain1(i) = abs(P1(periods+1));
    phase1(i) = angle(P1(periods+1))*180/pi;    
end

end

