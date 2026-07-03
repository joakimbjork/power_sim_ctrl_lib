function [ g_xy ] = f_init(y_PQG,PAR1,PAR,YBUS)
% NOT USED
% 
% Initializes system by calling the function:
% [y_PQG,FVAL,EXITFLAG]=fsolve(@f_init,y_PQG,option_fsolve,PAR1,PAR,YBUS);
% iterates until g_xy < 0 (or < tolerance value)

% YBUS - system admittance matrix

% PAR1=[nbus;nslack;ng;Sbus;Gbus];
nbus=PAR1(1); % Number of buses
nslack=PAR1(2); % Number of slack buses
ng=PAR1(3); % Number of generators (PU + slack buses)
Sbus=PAR1(4:3+nslack); % Index of slack buses
Gbus=PAR1(3+nslack+1:3+nslack+ng); % Index of generator buses

% PAR=[ANG0; VOLT0; PG0; QG0; PL0; QL0]; Initial values for
ANG0 = PAR(1:nbus); % Voltage phase angles
VOLT0 = PAR(nbus+1:2*nbus); % Voltage amplitudes
PG0 = PAR(2*nbus+1:3*nbus); % Injected active power from generators
QG0 = PAR(3*nbus+1:4*nbus); % Injected reactive power from generators
PL0 = PAR(4*nbus+1:5*nbus); % Active bus loads
QL0 = PAR(5*nbus+1:6*nbus); % Reactive bus loads

%% Algebraic variables, updates initial values
%ANG (all except slack), VOLT (non generator buses)
ANG = y_PQG(1:nbus-nslack); % Voltage phase angle at PU + PQ buses
VOLT = y_PQG(nbus-nslack+1:2*nbus-nslack-ng); % Voltage amplitude at PQ buses
PG = y_PQG(2*nbus-nslack-ng+1:2*nbus-ng); % Injected active power at slack bus
QG = y_PQG(2*nbus-ng+1:2*nbus); % Injected reactive power at slack + PU bus
% Take into acount which buses are slack, PU and PQ
bus=1:nbus;
not_Sbus = bus(~ismember(bus,Sbus)); % Index of all PU + PQ buses
not_Gbus = bus(~ismember(bus,Gbus)); % Index of all PU buses
% Update variables
ANG0(not_Sbus) = ANG0(not_Sbus)+ANG;
VOLT0(not_Gbus) = VOLT0(not_Gbus)+VOLT;
PG0(Sbus)=PG0(Sbus)+PG;
QG0(Gbus)=QG0(Gbus)+QG;

%% Calculate load flow with updated variables
b=-imag(YBUS); % b contains b_kj given in (A.21)-(A.22)
g=real(YBUS);
Vmat = VOLT0*VOLT0';
ANGmat = ANG0*ones(1,nbus);
ANGmat = ANGmat-ANGmat';
P_line = g.*(VOLT0.^2)-Vmat.*(g.*cos(ANGmat)+b.*sin(ANGmat));
Q_line = -b.*(VOLT0.^2)-Vmat.*(g.*sin(ANGmat)-b.*cos(ANGmat));

%% Active and reactive powerbalance with the updated load flow
P_balance =  sum(P_line,2) + PL0 - PG0;
Q_balance =  sum(Q_line,2) + QL0 - QG0;
g_xy=[P_balance; Q_balance];
%Iterates until active and reactive power is balanced at each bus
end

