function [tile, h1] = tiledLatex(y,x)
if nargin <= 1
    tile = tiledlayout('flow');
elseif nargin == 2
    tile = tiledlayout(y,x);
end
    tile.TileSpacing = 'compact'; % 'loose' (default) | 'compact' | 'tight' | 'none'
    tile.Padding = 'tight'; % 'loose' (default) | 'compact' | 'tight'
    tile.Title.Interpreter = 'latex';
    tile.Title.FontSize = 12;
    tile.Subtitle.Interpreter = 'latex';
    tile.Subtitle.FontSize = 12;
    tile.XLabel.Interpreter = 'latex';
    tile.YLabel.Interpreter = 'latex';
    
    h1 = nexttile(1);
% h1 = nexttile(1);

% h2 = nexttile(2);

end