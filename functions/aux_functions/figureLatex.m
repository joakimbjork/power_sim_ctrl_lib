function [hfig, names, co] = figureLatex(figWidth,figHeight)
% [hfig, names, co] = figureLatex(5,5); % figureLatex(figWidth,figHeight)
% tile_fig = tiledLatex(2,1);
%                     
% Skapad 2022-02-11
% Joakim Björk, joakim.bjork@svk.se

hfig = figure;
names = cell(0); % To store names

set(0,'DefaultTextInterpreter','latex',...
    'DefaultTextFontSize',10,...
    'DefaultAxesFontName','Times',...
    'DefaultAxesFontSize',10,...
    'DefaultLineMarkerSize',6)
set(gca,'TickLabelInterpreter','latex')
set(hfig,'DefaultLineLineWidth',0.8*1.5)

set(hfig,'units','inches',...
    'NumberTitle','off');
pos = get(hfig,'position');

%% Figure size 3.5 (inches) good for two-column article
if ~exist('figWidth','Var')
    figWidth = 3.5;
end
if ~exist('figHeight','Var')
    figHeight = 3.5/sqrt(2);
end
set(hfig,'position',[pos(1:2),figWidth,figHeight])

% movegui % Move figure on screen

%% Line Color
default_2D_colors = [   0, 0.4470, 0.7410;
                        0.8500, 0.3250, 0.0980;
                        0.9290, 0.6940, 0.1250;
                        0.4940, 0.1840, 0.5560;
                        0.4660, 0.6740, 0.1880;
                        0.3010, 0.7450, 0.9330;
                        0.6350, 0.0780, 0.1840];                       
set(groot,'defaultAxesColorOrder', default_2D_colors)

co = default_2D_colors;
% Alt: co = get(gca, 'ColorOrder');

%% Label
% ylabel('Amplitude')
% xlabel('Time')
% title('Step')
% set(gca,'YTick',[49.8, 49.9, 50])

%% Legends with Latex text interpreter
% l = legend('$\Delta f_1$', '', '$\Delta f_2$');
%         set(l,'Interpreter','LaTeX');
%         set(l,'Box','off')
%         set(l,'location','best')
%         set(l,'FontSize',9)
%         set(l,'Orientation','vertical')

