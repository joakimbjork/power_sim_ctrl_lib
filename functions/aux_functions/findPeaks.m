function [max_y, idx_y] = findPeaks(t,y,f)
% Skapad 2022-02-11
% Joakim Björk, joakim.bjork@svk.se


dt = t(2)-t(1); % Tidsupplösning i sekunder
samples_per_period = round(1/(f*dt)); % Antal datapunkter per period
number_of_periods = floor(length(t)/samples_per_period); % Antal perioder

max_y = zeros(number_of_periods,1);
idx_y = zeros(number_of_periods,1);
for i = 1:number_of_periods
    period_start = round((i-1)/(f*dt));
    interval = period_start + (1:samples_per_period);
    [max_y(i), idx_y(i)] = max(y(interval));   
    idx_y(i) = idx_y(i) + period_start;
end
end

