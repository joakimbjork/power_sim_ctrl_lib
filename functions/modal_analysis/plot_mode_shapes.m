function [fig, l, tile] = plot_mode_shapes(out_c,y_dim,x_dim, names, size_fig)
if exist('size_fig','VAR')
    [fig,~] = figureLatex(size_fig(1),size_fig(2));
else
    [fig,~] = figureLatex(5,5);
end
 
n_modes = length(out_c.e);

co = get(gca, 'ColorOrder');
n_colors = size(co,1);
m = length(out_c.v(:,1));
for i = 0:floor(m/size(co,2))
    co = [co;co];   
end


for i = 1:y_dim*x_dim
    if i <= n_modes
%         subplot(y_dim,x_dim,i) 
        if i == 1
        tile = tiledLatex(y_dim,x_dim); 
        tile.TileSpacing = 'loose';
        h = cell(0);
        end
      
        h{i} = nexttile(i); 
        w = out_c.v(:,i);
        w_abs = max(abs(w));
        w0 = ceil(w_abs*10)/10;
        c0 = compass(w0); hold all
        set(c0, 'Visible', 'on','Linestyle', 'none', 'Marker', 'none')
        c = compass(w); 
        text_string = [num2str(round(out_c.f(i),2)), ' Hz',...
                       ' (' num2str(round(out_c.d(i),3)*100),' \%)'];
        if y_dim*x_dim == 1
            text(0,0,text_string,'Units','normalized')   
        else
            text(0,-0.15,text_string,'Units','normalized')
        end
            
        for j = 1:length(out_c.v(:,i))           
            c1 = c(j);
            c1.LineWidth = 1.5;
            c1.Color = co(j,:);
            if j>n_colors
                c1.LineStyle = ':';
            end
        end
    end

end
if exist('names','VAR')
    N = length(names);
    name_ = cell(N+1,1);
    name_{1} = '';
    for i = 1:N
        name_{i+1} = names{i};
    end
    l = legendLatex(fig, name_); % set(l,'location','southeast')
    l.Layout.Tile = 'east';
end
end

