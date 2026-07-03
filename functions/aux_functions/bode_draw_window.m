function bode_draw_window(freq_min,freq_max)

% nexttile(1)
% yline(1);
% nexttile(2)
% yline(0);
for i = 1:2
nexttile(i)
ylims = ylim;
% dy = ylims(2)-ylims(1);
% ylims(1) = ylims(1)+dy*0.05;
% ylims(2) = ylims(2)-dy*0.05;
patch([freq_min freq_max freq_max freq_min],...
    [ylims(1) ylims(1) ylims(2) ylims(2)]*1,...
    [1,1,1]*0.6,'edgecolor',[1,1,1]*0.6,'FaceAlpha', 0.15    );
% p_ = patch([freq_min freq_max freq_max freq_min],...
%     [ylims(1) ylims(1) ylims(2) ylims(2)]*1,...
%     [1,1,1]*0.9,'edgecolor',[1,1,1]*0.9);
ylim(ylims)
% set(gca, 'Layer', 'top')
% uistack(p_, 'bottom')
end
end

