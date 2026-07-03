function l = legendLatex(fig, names, target)
%LEGENDLATEX
% Skapad 2022-02-11
% Joakim Björk, joakim.bjork@svk.se
%
% %%%% Example1 %%%%
% nexttile(1)
% l = legendLatex(fig,{'$y_1$','$y_2$'}); 
% set(l,'location','east')
% 
% %%%% Example2 %%%%
% names = cell(0);
% names{1} = '$y_1$';
% names{2} = '$y_2$';
% nexttile(1)
% l = legendLatex(fig,names); 
% set(l,'location','east')

if ~isempty(fig)
    figure(fig)
end

if nargin == 2
    l = legend(names);
elseif nargin == 3
    l = legend(target, names);
end
set(l,'Interpreter','LaTeX');
set(l,'Box','on')
set(l,'location','best')
set(l,'FontSize',9)
set(l,'Orientation','vertical')