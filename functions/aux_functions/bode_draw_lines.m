function bode_draw_lines(x_loc,names,type)
if nargin <= 2
    type = 1;
end
m = length(x_loc);
switch type
    case 1
        for i = 1:2
            nexttile(i)
            ylims = ylim;
            for idx = 1:m
                plot(x_loc(idx)*[1;1], ylims','color',[1,1,1]*0.6,'linewidth',0.4)
            end
            if i == 1
                if nargin >= 2
                    assert(length(names)==m)

                    h = text(x_loc, min(ylims)*ones(1,m),...%-abs(mean(ylims))/2,...
                        names,'HorizontalAlignment','center',...
                        'VerticalAlignment','middle');
                    set(h,'BackgroundColor','w',...
                        'Edgecolor', [1,1,1]*0.6,...
                        'Rotation',45);
                end
            end
        end
    case 2
        for i = 1:2
            nexttile(i)
            ylims = ylim;
            for idx = 1:m
                plot(x_loc(idx)*[1;1], ylims','color',[1,1,1]*0.6,'linewidth',0.4)
            end
            if i == 2
                if nargin >= 2
                    assert(length(names)==m)

                    h = text(x_loc, (max(ylims)+0.1*abs(diff(ylims)))*ones(1,m),...%-abs(mean(ylims))/2,...
                        names,'HorizontalAlignment','center',...
                        'VerticalAlignment','middle');
                    set(h,'BackgroundColor','w',...
                        'Edgecolor', [1,1,1]*0.6,...
                        'Rotation',90);
                end
            end
        end
    case 3
        for i = 1:2
            nexttile(i)
            ylims = ylim;
            for idx = 1:m
                plot(x_loc(idx)*[1;1], ylims','color',[1,1,1]*0.6,'linewidth',0.4)
            end
            if i == 2
                if nargin >= 2
                    assert(length(names)==m)
                    
                    h = text(x_loc, (max(ylims)+0.3*abs(diff(ylims)))*ones(1,m),...%-abs(mean(ylims))/2,...
                        names,'HorizontalAlignment','center',...
                        'VerticalAlignment','middle');
                    set(h,'BackgroundColor','w',...
                        'Edgecolor', [1,1,1]*0.6,...
                        'Rotation',90);
                end
            end
        end
end

