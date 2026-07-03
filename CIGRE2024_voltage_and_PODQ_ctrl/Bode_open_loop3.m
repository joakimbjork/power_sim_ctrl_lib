
opt_CIGRE2024;
[fig,name,co] = figureLatex(4,3.5);
bodeopt = struct(); 
f_comp = 1;
% bodeopt.FreqUnits = 'Hz';
% omega_lim = log10( [1e-3,1e2]*2*pi );
% omega_lim = log10( [1e-6,1e2]*2*pi );
% bodeopt.omega = logspace(omega_lim(1),omega_lim(2),10000);
bodeopt.omega_lim = [2*1e-6,1e2]*2*pi ;

bodeopt = bodeLatex(H*Kc*G,bodeopt);
bodeopt = bodeLatex(H*K*G,bodeopt);
bodeopt.linestyle = '--';
bodeopt = bodeLatex(H*K_POD*G,bodeopt);
% bodeopt = bodeLatex(H0*K_opt*G,bodeopt);

% names_ = {'$G(j\omega)$',...
%          '$L(j\omega)$, $\tau={}$75\,ms',...
%                    '$L(j\omega)$, $\tau={}$0\,ms'};

set(bodeopt.h1,'xticklabel',[])
bodeopt.tile.TileSpacing = 'compact';
nexttile(1)
ylim([10^-3,10^2])
nexttile(2)
ylim([-190,100])
yticks([-270,-180,-90,0,90])
yline(0)
yline(-180)

% bode_draw_window(0.25*2*pi/f_comp,1*2*pi/f_comp); % Illustrade region where we want damping
% bode_draw_lines([a1 a2 wE 1/TD]/(2*pi),{'$\alpha_1$', '$\alpha_2$', '$\omega_E$', '$1/\tau$'}); % Illustrate controller poles and zeros
bode_draw_lines([a1,a2,b1,b2,wE,1/TD]/f_comp); % Illustrate controller poles and zeros
bode_draw_lines_top([0.25*2*pi,1*2*pi]/f_comp,{'$\omega_1$','$\omega_2$'})

nexttile(1)
p2 = loglog(bodeopt.omega_lim,[G.D,G.D],'-.','color',co(4,:));

for i = 1:6
if i == 1
    nexttile(2)
    names = {'$a_1$'};
    freq = [a1]/f_comp;
    ylims = ylim; ypos = max(ylims);
    c = [1,0.45,0.45]*1;
elseif i == 2
    nexttile(2)
    names = {'$a_2$'};
    freq = [a2]/f_comp;
    ylims = ylim; ypos = max(ylims);
    c = [1,0.45,0.45]*1;  
elseif i == 3
    nexttile(1)
    names = {'$b_1$'};
    freq = [b1]/f_comp;
    ylims = ylim; ypos = min(ylims);
    c = [1,0.45,0.45]*1; 
elseif i == 4
    nexttile(1)
    names = {'$b_2$'};
    freq = [b2]/f_comp;
    ylims = ylim; ypos = min(ylims);
    c = [1,0.45,0.45]*1;  
elseif i == 5
    nexttile(1)
    names = {'$\omega_E$'};
    freq = [wE]/f_comp;
    ylims = ylim;
    ypos = min(ylims);
    c = [0.84,0.89,0.96]*1;
elseif i == 6
    nexttile(2)
    names = {'$1/\tau$'};
    freq = [1/TD]/f_comp;
    ylims = ylim;
    ypos = max(ylims);
    c = [0.84,0.89,0.96]*1;
end        
    m = length(freq);   
    h = text(freq, ypos*ones(1,m),names,'HorizontalAlignment','center','VerticalAlignment','bottom');
    set(h,'BackgroundColor',c,'Edgecolor', [1,1,1]*0.6,'Rotation',0);
end

names_ = {'Voltage control',...
          'POD-Q',...
          'Pure POD-Q'};
nexttile(1);
l=legendLatex(gcf,'$1/S_\mathrm{SCP}$',p2);
set(l,'location','west')
nexttile(2);
l=legendLatex(gcf,names_);
set(l,'location','southwest')

saveLatex(fig,'bode_PODQ',fig_save_opt)
