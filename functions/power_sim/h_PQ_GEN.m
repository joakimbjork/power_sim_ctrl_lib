function [y_PQ]=h_PQ_GEN(xy0,PAR)
ng=PAR.ng; %Number of generators
nbus=PAR.nbus; %Number of buses
nx = PAR.nx;
ns = PAR.ns;
Gbus=PAR.Gbus; %Index of generator buses

% PL0=PAR.PL0; %Initial active loads
% QL0=PAR.QL0; %Initial reactive loads
% U0=PAR.U0; %Initial bus voltage amplitudes
% ANG0 = PAR.ANG0; %Initial bus voltage phase angles

%% Generator info
XDP=PAR.XDP; %d-axis transient reactance
XQP=PAR.XQP; %q-axis transient reactance
BDP = 1./XDP; %Generator transient susceptance(1./XDP)
XD=PAR.XD; %d-axis synchronous reactance
% M=PAR.M; %Generator innertia constant
% D=PAR.D; %Generator damping constant
% TDOP=PAR.TDOP; %d-axis transient open-circuit time constant

if ns <= 3
    XQP = XDP; % Transient saliency is neglected
    XQ = XQP; % No rotor windings on q-axis    
elseif ns >= 4
    XQP = PAR.XQP; %q-axis transient reactance
    XQ = PAR.XQ; %q-axis synchronous reactance
    TQOP = PAR.TQOP; %q-axis transient open-circuit time constant
%     if ns == 6
%         XDPP = PAR.XDPP; %d-axis subtransient reactance
%         TDOPP = PAR.TDOPP; %d-axis subtransient open-circuit time constant
%         XQPP = PAR.XQPP; %q-axis subtransient reactance
%         TQOPP = PAR.TQOPP; %q-axis subtransient open-circuit time constant
%         XL = PAR.XL; % Stator leakage reactance
%     end
end

%% States and algebraic variables
x0=xy0(1:nx); %Current state variable values
y0=xy0(nx+1:nx+2*nbus); %Current algebraic variable values
DELTA=x0(1:ng);
% OMEGA=x0(ng+1:2*ng);
if ns >= 3
    EQP=x0(2*ng+1:3*ng);
else
    EQP = PAR.EQP;
end

if ns >= 4
    EDP=x0(3*ng+1:4*ng);
else
    EDP = PAR.EDP;
end

ANG=y0(1:nbus);
VOLT=y0(nbus+1:2*nbus);

ANGG = ANG(Gbus);
UG = VOLT(Gbus);
% UG0 = U0(Gbus);
%% Active power output of generators
% Udq = UG.*exp(1j*(ANGG-DELTA+pi/2)); = 1j*UG.*exp(1j*(ANGG-DELTA)) 
% Ud = real(Udq);
% Uq = imag(Udq);
Ud = -UG.*sin(ANGG-DELTA);
Uq = UG.*cos(ANGG-DELTA);
Iq = (Ud-EDP)./XQP;
Id = -(Uq-EQP)./XDP;

% Alternative 1: Calculation of power injections
PE = Id.*Ud + Iq.*Uq;
QE = Id.*Uq - Iq.*Ud; 
% PE = EDP.*Id + EQP.*Iq + (XQP-XDP).*Id.*Iq;
% QE = EQP.*Id - EDP.*Iq - XDP.*Id.^2 - XQP.*Iq.^2; 

% Alternative 2: Only for one-axis or classic model
% PE=EQP.*UG.*sin(DELTA-ANGG)./XDP; %Active power output of generators
% QE=-(UG.^2-UG.*EQP.*cos(ANGG-DELTA))./XDP; %Rective power injection at generator bus


%% Output
y_PQ = [PE; QE];

