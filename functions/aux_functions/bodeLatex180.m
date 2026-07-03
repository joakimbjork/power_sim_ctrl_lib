function [h1, h2, tile] = bodeLatex180(sys,omega,h1,h2,color,linestyle)
% Skapad 2022-02-11
% Joakim Björk, joakim.bjork@svk.se
if exist('omega','Var') && ~isempty(omega)
    [mag,phase] = bode(sys,omega);
else
    [mag,phase,omega] = bode(sys);
end

if exist('color','Var')
    co = true;
else
    co = false;
end
if exist('linestyle','Var');
    ls = linestyle;
else
    ls = '-';
end

mag = squeeze(mag);
phase = squeeze(phase);
% if exist('wrap','Var')
phase = wrapTo180(phase);


if ~exist('h1','Var')
    tile = tiledlayout(2,1); 
    tileLatex;
    h1 = nexttile;    
%     h1 = subplotLatex(2,1,1);
else
    % För att fortsätta plotta i samma figur behöver vi ha med handtag
    axes(h1)
end
if co 
    loglog(omega,mag,ls,'color',color); hold all
else
    loglog(omega,mag); hold all
end
ylabel('Amplitud')
xlim([omega(1),omega(end)])

if ~exist('h2','Var')
%     h2 = subplotLatex(2,1,2);
    h2 = nexttile;
else
    % För att fortsätta plotta i samma figur behöver vi ha med handtag
    axes(h2)
end
if co
    semilogx(omega,phase,ls,'color',color); hold all
else
    semilogx(omega,phase); hold all
end
% Hårdkodade gränser för att ge snygga y-värden
ylims = get(h2,'ylim');
ydiff = abs(diff(ylims));
if ydiff < 45; yint = 10;
elseif ydiff < 100; yint = 30;
elseif ydiff < 190; yint = 45;
elseif ydiff < 400; yint = 90;
elseif ydiff < 1000; yint = 180;
elseif ydiff < 2000; yint = 360;
else yint = 0;
end
if yint~=0
    set(h2,'YTick',-10*yint:yint:10*yint);
end

ylabel('Fas [$^\circ$]')
xlabel('Vinkelfrekvens [rad/s]')
xlim([omega(1),omega(end)])

% figure(h1)
end

