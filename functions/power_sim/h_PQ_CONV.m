function [y_Pe]=h_y(xy0,PAR)
ng=PAR.ng; %Number of generators
nbus=PAR.nbus; %Number of buses
nx = PAR.nx;
ns = PAR.ns;
Gbus=PAR.Gbus; %Index of generator buses

% PL0=PAR.PL0; %Initial active loads
% QL0=PAR.QL0; %Initial reactive loads
% U0=PAR.U0; %Initial bus voltage amplitudes
% ANG0 = PAR.ANG0; %Initial bus voltage phase angles

%% States and algebraic variables
x0=xy0(1:nx); %Current state variable values
y0=xy0(nx+1:nx+2*nbus); %Current algebraic variable values


ANG=y0(1:nbus);
VOLT=y0(nbus+1:2*nbus);


%% Active power of converters
PC = [];
QC = [];
Cbus = [];

if isfield(PAR,'CONV')
    Cbus = PAR.CONV.Cbus;
    U = VOLT(Cbus);
    ANGC = ANG(Cbus);
    UC = U.*exp(1j*ANGC);
    if PAR.CONV.type == 1 % --> Perfect alignment
        DELTAC = ANGC;
    else % Grid forming/following gets phase angle from integrator block "G.phase_ctrl"
        [~, DELTAC] = ss_object(0, PAR.CONV.phase_ctrl, x0, 0*PAR.CONV.DELTAC0, PAR.CONV.DELTAC0); % Output is DELTAC
    end

    Udq = UC.*exp(1j*(-DELTAC));
    Ud = real(Udq);
    Uq = imag(Udq);

    
    % This requires Idq control blocks to be strictly proper
    [~,~,C1,~] = ssdata(PAR.CONV.Id_ctrl.sys); %
    x1 = x0(PAR.CONV.Id_ctrl.idx_nx); % pick out states
    Id = C1*x1+PAR.CONV.Id0;
    [~,~,C1,~] = ssdata(PAR.CONV.Iq_ctrl.sys); %
    x1 = x0(PAR.CONV.Iq_ctrl.idx_nx); % pick out states
    Iq = C1*x1+PAR.CONV.Iq0;
    
    % Power injected at network node
    PC = Iq.*Uq + Id.*Ud;
    QC = Id.*Uq - Iq.*Ud;
end



%% Output
y_Pe = [PC; QC];

