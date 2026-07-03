function names = plotSineTest(t,y,u,omega,gain, phase, tlim, ty, tu)
% Skapad 2022-02-11
% Joakim Björk, joakim.bjork@svk.se

names = cell(0);
% Rita fönster för beräkningen av fasförskjutning
patch([tu ty ty tu],[-1 -1 1 1]*max(1,gain),[1,1,1]*0.8,'edgecolor','none'); hold on
names{1} = '';

% Rita signaler
plot(t,u); hold all
plot(t,y);
names{2} = '$u(t)$';
names{3} = '$y(t)$';

% Rita beräknad förstärkning och fasförskjutning
yline(gain,'--')
names{4} = ['$y_0/u_0=$ ', num2str(round(gain,1))];
xline(ty); xline(tu)
names{5} = ['$\theta=$ ', num2str(round(phase*180/pi)),'$^\circ$'];

box on
xlim(tlim)
ylim([-1,1]*max(1,gain)*1.1)
end

