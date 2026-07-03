G1_ = minreal(G1(y_idx,u_idx));
G2_ = minreal(G2(y_idx,u_idx));
if y_idx == 4
    G1_ = s/(s*0.001+1)*G1_;
    G2_ = s/(s*0.001+1)*G2_;
end


bodeopt_ = bodeLatex(G1_,bodeopt);
bodeopt_.linestyle = '--';
bodeopt_ = bodeLatex(G2_,bodeopt_);
% p1 = bodeopt_.h1.Children;
% p2 = bodeopt_.h2.Children;
plot_residues_2;
% bode_draw_window(0.25*2*pi,1*2*pi);
bode_draw_lines_bot([0.25*2*pi,1*2*pi]/f_comp,{'$\omega_1$','$\omega_2$'})

name{1} = '$P_\mathrm{line} = 0.5\,\mathrm{p.u.}$';
name{2} = '$P_\mathrm{line} = -0.5\,\mathrm{p.u.}$';

nexttile(2);
% ylim([-360,180])
l = legendLatex(fig,name); set(l,'location','southwest'); %set(l,'position',[0.2,0.23,0.42,0.15]);
% nexttile(2); l = legendLatex(fig,name,p_res2); set(l,'location','southwest');
% set(gca,'FontSize',fontsize)
% nexttile(2)
% set(gca,'FontSize',fontsize)

set(bodeopt_.h1,'xticklabel',[])
bodeopt_.tile.TileSpacing = 'compact';
% bodeopt_.tile.Padding = 'compact'; % 'loose' (default) | 'compact' | 'tight'

if (u_idx == 8 && y_idx == 8) || (u_idx == 4 && y_idx == 4)
    sys = 1/tf(SCP);
    if y_idx == 4
        sys = s/(s*0.001+1)*sys;    
    end
    xlims = xlim;
    [mag,phase,omega] = bode(sys,xlims);  
    mag = squeeze(mag);
    nexttile(1)
    p2 = plot(omega,mag,'-.','color',co(4,:));
    if y_idx == 4
        l=legendLatex(gcf,'$\omega/S_\mathrm{SCP}$',p2);   
    else
        l=legendLatex(gcf,'$1/S_\mathrm{SCP}$',p2);
    end
    
    set(l,'location','southwest')
end
