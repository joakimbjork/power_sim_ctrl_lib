function bode_draw_lines_bot(x_loc,names)
nexttile(1)
yline(1);
nexttile(2)
yline(0);
m = length(x_loc);
for i = 1:2
    nexttile(i)
    ylims = ylim;
    for idx = 1:m
        plot(x_loc(idx)*[1;1], ylims','k','linewidth',0.4)
    end
    if i == 1
        h = text(x_loc, min(ylims)*ones(1,m),...%-abs(mean(ylims))/2,...
            names,'HorizontalAlignment','center','VerticalAlignment','baseline');
        set(h,'BackgroundColor','w',...
            'Edgecolor', 'k',...
            'Rotation',0);
    end
end