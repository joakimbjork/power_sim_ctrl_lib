mode = modal_shape2(G1_);
f_comp = 1;
nexttile(1)
p_res1(1) = plot(abs(mode.e)/f_comp,mode.r_abs/max(mode.r_abs),'*','color',co(1,:)); hold all
nexttile(2)
p_res2(1) = plot(abs(mode.e)/f_comp,mode.r_ang,'*','color',co(1,:));
mode = modal_shape2(G2_);
nexttile(1)
p_res1(2) = plot(abs(mode.e)/f_comp,mode.r_abs/max(mode.r_abs),'*','color',co(2,:)); hold all
nexttile(2)
p_res2(2) = plot(abs(mode.e)/f_comp,mode.r_ang,'*','color',co(2,:));