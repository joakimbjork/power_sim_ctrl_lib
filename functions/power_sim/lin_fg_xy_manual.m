function [G, YBUS, Y0, fig] = lin_fg_xy_manual(PAR, draw_graph, scale)
% Joakim Björk, joakim.bjork@svk.se, 2022-03-03
% Linearization method from:
% Control Limitations due to Zero Dynamics in a Single-Machine Infinite Bus Network,
% Joakim Björk, Karl Henrik Johansson, 
% IFAC-PapersOnLine, Volume 53, Issue 2, 2020, Pages 13531-13538
% URL: https://doi.org/10.1016/j.ifacol.2020.12.796


if ~isfield(PAR,'input')
    input = 'constant_P_and_Q';
else
    input = PAR.input;
end

ng = PAR.ng;
nbus = PAR.nbus;
Zload = PAR.PL0-1j*PAR.QL0;
V = PAR.U0;
EQP = PAR.EQP;
DELTA = PAR.xy0(1:ng);
ANG = PAR.ANG0;

XDP = PAR.XDP;
XD = PAR.XD;
Xdel = XD-XDP;

dD = diag(PAR.D);
dM = diag(PAR.M);
dE = diag(EQP);
TDOP = PAR.TDOP;
for i = 1:length(TDOP)
    TDOP(i) = min(TDOP(i),1e9); % TDOP can't be inf for the manual linearization
end
dTDOP = diag(TDOP);
dT = dTDOP*inv(diag(Xdel))*dE;

dV = diag(V);

%% Admittance matrix
Y11=diag(1./(1j*XDP));
Y12=-diag(1./(1j*XDP));
Ygen = diag(1./(1j*Xdel));

if any(PAR.mp ~= 2) || any(PAR.mq ~= 2)
    if any(PAR.PL0 ~= 0) || any(PAR.QL0 ~= 0)
        disp('OBS! Only constant impedance loads are captured by the manual linearization. For accuracy, use numeric linearization (lin_fg_xy.m).')
    end
end
Yload=diag(Zload./(V.^2));

Y21=Y12;
if nbus > ng
    Y12=[Y12, zeros(ng,nbus-ng)];
    Y21=[Y21 ; zeros(nbus-ng,ng)];
end

Y22=PAR.YBUS;
Y22(1:ng,1:ng) = Y22(1:ng,1:ng) + Y11;



Y11 = Y11 + Ygen;
Y22 = Y22 + Yload;
Y0 = [Y11, Y12;
    Y21, Y22];

YBUS = Y22;
YBUS(1:ng,1:ng) =  YBUS(1:ng,1:ng) + diag(1./(1j*XDP));

% Y_RNM=Y11-Y12*inv(Y22)*Y21; %RNM bus admittance matrix

%% Weighted admittance matrix
U = [EQP.*exp(1j*DELTA);V.*exp(1j*ANG)];
Y = diag(U)*conj(Y0)*diag(U');
% S = sum(Y,2); % Complex power injections

%% State space model
% Y = | Y11 Y12 |
%     | Y21 Y22 |
Y11 = Y(1:ng,1:ng);
Y12 = Y(1:ng,ng+1:end);
Y21 = Y(ng+1:end,1:ng);
Y22 = Y(ng+1:end,ng+1:end);

%%
YD = -inv(Y22);
YB = Y12*YD;
YC = YD*Y21;

Yred = Y11 + Y12*YD*Y21;
Ysh = diag(sum(Yred,2));
YA = Yred - Ysh;

W1 = blkdiag(eye(ng),inv(dM),inv(dT));
W2 = blkdiag(eye(ng),eye(ng),inv(dE));
Wy = blkdiag(eye(nbus),dV);

A = [zeros(ng), eye(ng),    zeros(ng);
    -imag(YA), -dD,        -real(YA+2*Ysh);
    real(YA),  zeros(ng), -imag(YA+Ysh)];
A = W1*A*W2;

if strcmp(input,'constant_P_and_Q')
    B = [zeros(ng,nbus),  zeros(ng,nbus);
        real(YB),       -imag(YB);
        imag(YB),        real(YB)];
    B = W1*B;
elseif strcmp(input,'UREF')
    B = [zeros(ng);
        zeros(ng);
        eye(ng)*inv(dTDOP)];
end

C = [real(YC), zeros(nbus,ng), -imag(YC);
    imag(YC), zeros(nbus,ng),  real(YC)];
C = Wy*C*W2;

if strcmp(input,'constant_P_and_Q')
    D = [imag(YD), real(YD);
        -real(YD), imag(YD)];
    D = Wy*D;
elseif strcmp(input,'UREF')
    D = zeros(2*nbus,ng);
end

%%
G = ss(A,B,C,D);

if PAR.ns >= 3
    disp('The output is a manually linearized network model with one-axis machines')
elseif PAR.ns == 2
    disp('The output is a manually linearized network model with classical machines')
    idx = (2*ng)+1:3*ng;
    G = modred(G,idx,'truncate');
end

ng_inf = ng;
if PAR.infbus ~=0 % Infbus modifications
    if PAR.ns==2
        idx = PAR.infbus:ng:2*ng;
    else
        idx = PAR.infbus:ng:3*ng;
    end
    G = modred(G,idx,'truncate');

    
    
    idx = PAR.infbus;    
    ng_inf = ng-length(idx);
% %     Y(idx+ng,:) = [];
% %     Y(:,idx+ng) = [];
% %     Y0(idx+ng,:) = [];
% %     Y0(:,idx+ng) = [];    
    Y(idx,:) = [];
    Y(:,idx) = [];
    Y0(idx,:) = [];
    Y0(:,idx) = [];
%     ng = ng-length(idx);
end



if nargin >= 2 && draw_graph ~= 0
    if draw_graph == 1
        disp('Drawing network with weighted impedances')
        g = graph(imag(Y),'omitselfloops','lower'); % Does 'lower' or 'upper' option make any difference here?
    else
        disp('Drawing network with impedances')
        g = graph(imag(conj(Y0)),'omitselfloops'); %
    end
    if nargin == 2
        scale = 1;        
    end
    
    names = cell(0);
    e = -scale./g.Edges.Weight;
    for i = 1:length(e)
        names{i} = ['j ', num2str(round(e(i),2))];
    end
    
    m = size(g.Nodes,1);
    name_nodes = cell(m,1);
    colors = zeros(m,3);
    %     marker_size = zeros(m,1);
    %     markers = cell(m,1);
    for i = 1:m
        if i <= ng_inf
            if any(i == PAR.infbus)
                name_nodes{i} = ['INF-GEN ', num2str(i)];
                colors(i,:) = [0 0 0];
            else
                name_nodes{i} = ['GEN ', num2str(i) ];
                % name_nodes{i} = ['GEN ', num2str(i),...
                %                  ', ', num2str(round(EQP(i),2)), '(',num2str(round(DELTA(i)*180/pi,2)) , '^o)' ];
                colors(i,:) = [0.8500 0.3250 0.0980];
            end
            %         markers{i} = 'o';
            %         marker_size(i) = 7;
        else
            
            %         markers{i} = '|';
            %         marker_size(i) = 50;
            if any(i-ng_inf == PAR.infbus)
                name_nodes{i} = ['INF-BUS ', num2str(i-ng_inf)];
                colors(i,:) = [0 0 0];
            else
                name_nodes{i} = ['BUS ', num2str(i-ng_inf)];
                colors(i,:) = [0 0.4470 0.7410];
            end
        end
    end
    
    fig = figureLatex(6,2);
    p = plot(g,'EdgeLabel',names,...
        'NodeLabel',name_nodes,...
        'NodeColor',colors);
    %            'Marker',markers,...
    %            'MarkerSize',marker_size);
    ax =gca;
    % set(ax,'Visible','off')
    camroll(90)
else
    fig = [];
end
