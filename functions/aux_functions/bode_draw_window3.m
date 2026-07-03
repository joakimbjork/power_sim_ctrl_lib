function bode_draw_window3(freq_min,freq_max,phase_min,phase_max,color)
ylims = [phase_min, phase_max];
y1 = ylims(1);
y2 = ylims(2);
d = [1,1,1]-color;
color_d = [227,244,215]/255;
patch([freq_min freq_max freq_max freq_min],...
    [ylims(1) ylims(1) ylims(2) ylims(2)]*1,...
    color_d,'edgecolor',color);
% plot([freq_min, freq_min, freq_max, freq_max],[y1, y2, y2, y1],'color',color)

