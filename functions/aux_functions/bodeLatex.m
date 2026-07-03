function [h1, h2, tile] = bodeLatex(sys,opt,h1,h2,color,linestyle)
%BODELATEX
% [opt] = bodeLatex(sys,opt)
% Skapad 2022-02-11
% Joakim Björk, joakim.bjork@svk.se
%
% %%%% Example %%%%
% [fig,~,co] = figureLatex(5,3.5);
% bodeopt = struct();
% % bodeopt.wrap = true;
% % bodeopt.FreqUnits = 'Hz';
% bodeopt.omega_lim = [5e-3,5e2] ;
%
% bodeopt = bodeLatex(G1,bodeopt); hold all
% bodeopt = bodeLatex(G2,bodeopt); hold all


name_xlabel = 'Angular frequency [rad/s]';
x_scale = 1;
name_ylabel_top = 'Magnitude';
name_ylabel_bot = 'Phase [$^\circ$]';
adjust = true;
plot_phase = true;

if nargin > 1
    if isstruct(opt)
        if isfield(opt,'omega')
            omega = opt.omega;
        end
        if isfield(opt,'h1')
            h1 = opt.h1;
        end
        if isfield(opt,'h2')
            h2 = opt.h2;
        end
        if isfield(opt,'color')
            color = opt.color;
        end
        if isfield(opt,'linestyle')
            linestyle = opt.linestyle;
        end
        if isfield(opt,'adjust')
            adjust = opt.adjust;
        end
        if isfield(opt,'norm_gain')
            norm_gain = opt.norm_gain;
        end
        if isfield(opt,'wrap')
            wrap = opt.wrap;
        end
        if isfield(opt,'omega_lim')
            omega_lim = opt.omega_lim;
            limObj = {omega_lim(1), omega_lim(2)};
        end
        if isfield(opt,'FreqUnits')
            if strcmp(opt.FreqUnits,'Hz')
                x_scale = 1/(2*pi);
                name_xlabel = 'Angular frequency [Hz]';
            end
        end
        if isfield(opt,'plot_phase')
            plot_phase = opt.plot_phase;
        end
    else
        omega = opt;
        opt = struct();
    end
end

if exist('limObj','Var')
    [mag,phase,omega] = bode(sys,limObj);
elseif exist('omega','Var') && ~isempty(omega)
    [mag,phase] = bode(sys,omega);
else
    [mag,phase,omega] = bode(sys);
end

if exist('x_scale','Var')
    omega = omega*x_scale;
end



mag = squeeze(mag);
phase = squeeze(phase);
if exist('wrap','Var') && wrap == true
    phase = wrapTo180(phase);
end

if exist('norm_gain','Var') && norm_gain == true
    mag = mag./mag(1,:);
end

if adjust
    adjust = 0;
    [~,f_max] = hinfnorm(pade(sys,6));
    f_diff = omega-f_max;
    [~,idx] = min(abs(f_diff));

    % G_max = freqresp(sys,f_max);
    % phase_max =
    if phase(idx) > 0
        phase_ = phase - 360;
        while abs(phase_(idx)) < abs(phase(idx))
            phase = phase_;
            phase_ = phase - 360;
            adjust = adjust - 1;
        end
    elseif phase(idx) < 0
        phase_ = phase + 360;
        while abs(phase_(idx)) < abs(phase(idx))
            phase = phase_;
            phase_ = phase + 360;
            adjust = adjust + 1;
        end
    end
    if adjust ~= 0
        disp(['bodeLatex: Adjusting phase with ', num2str(adjust),'*360 degrees'])
    end
end

if exist('h1','Var') && ~isempty(h1)
    axes(h1)
else
    if plot_phase
        tile = tiledLatex(2,1);
    else
        tile = tiledLatex(1,1);
    end
    opt.tile = tile;
    h1 = nexttile(1);
end


if exist('linestyle','Var') && ~isempty(linestyle)
    ls = linestyle;
else
    ls = '-';
end

if isfield(opt,'downsample')
    omega = downsample(omega,opt.downsample);
    mag = downsample(mag,opt.downsample);
    phase = downsample(phase,opt.downsample);
end





if exist('color','Var') && ~isempty(color)
    line = loglog(omega,mag,ls,'color',color); hold all
else
    line = loglog(omega,mag,ls); hold all
end
ylabel(name_ylabel_top)
if exist('omega_lim','Var') && ~isempty(omega_lim)
    xlims = [max(omega_lim(1)*x_scale,omega(1)),min(omega_lim(end)*x_scale,omega(end))];
else
    xlims = [omega(1),omega(end)];
end
xlim(xlims);

if plot_phase
    if exist('h2','Var') && ~isempty(h2)
        axes(h2)
    else
        h2 = nexttile(2);
    end
    if exist('color','Var') && ~isempty(color)
        line = semilogx(omega,phase,ls,'color',color); hold all
    else
        line = semilogx(omega,phase,ls); hold all
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
        set(h2,'YTick',-20*yint:yint:20*yint);
    end

    ylabel(name_ylabel_bot)
    xlabel(name_xlabel)
    xlim(xlims);
else
    xlabel(name_xlabel)
end

if nargout == 1
    h1_temp = h1;
    h1 = opt;
    h1.h1 = h1_temp;
    if plot_phase
        h1.h2 = h2;
    end
    h1.tile;
    h1.mag_out = mag;
    h1.phase_out = phase;
    h1.omega_out = omega;
    h1.line = line;
end
end

