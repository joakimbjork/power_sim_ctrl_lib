PAR.ws = ws;
if isfield(opt,'mp')
    PAR.mp = opt.mp;
else
    PAR.mp = 0;  % Active load characteristic
end
if isfield(opt,'mq')
    PAR.mq = opt.mq;
else
    PAR.mq = 0;  % Reactive load characteristic
end

if isfield(opt,'silent')
    PAR.silent = false; % Print information when loading model and linearizing
end
%% Generator data
%           1    2   3  4   5    6   7     8    9   10 
% GENDATA = Gen  H   D  xt  xdp  xd  Tdop  xqp  xq  Tqop 


PAR.MBASE = GENDATA(:,1)/Sb;
PAR.M=2*(GENDATA(:,2)).*PAR.MBASE/ws;
PAR.D=GENDATA(:,3);

PAR.XDP=(GENDATA(:,4)+GENDATA(:,5))./PAR.MBASE;
PAR.XD=(GENDATA(:,4)+GENDATA(:,6))./PAR.MBASE;
PAR.TDOP=GENDATA(:,7);

if size(GENDATA,2) == 7
    PAR.XQP = PAR.XDP; % Transient saliency is neglected
    PAR.XQ = PAR.XQP; % No rotor windings on q-axis  
    PAR.TQOP = PAR.TDOP;
else
    PAR.XQP=(GENDATA(:,4)+GENDATA(:,8))./PAR.MBASE;
    PAR.XQ=(GENDATA(:,4)+GENDATA(:,9))./PAR.MBASE;
    PAR.TQOP=GENDATA(:,10);
end

if PAR.ns == 6
%                 11    12     13    14      15
%  GENDATA = ...  Xdpp  Tdopp  Xqpp  Tqopp   Xl
    PAR.XDPP = (GENDATA(:,4)+GENDATA(:,11))./PAR.MBASE;
    PAR.TDOPP = GENDATA(:,12);
    PAR.XQPP = (GENDATA(:,4)+GENDATA(:,13))./PAR.MBASE;
    PAR.TQOPP = GENDATA(:,14);
    PAR.XL = (GENDATA(:,4)+GENDATA(:,15))./PAR.MBASE;
end

%% Load Flow
tole=1e-6;
maxiter=10;
flag_init=3;

%           1    2     3     4     5      6      7   8    9  10     
% BUSDATA = BUS  Type  Pgen  Qgen  Pload  Qload  YL  Ysh  V  Angle  

Gbus=find(BUSDATA(:,2)==1 | BUSDATA(:,2)==2); % Generator buses
PAR.nbus = length(BUSDATA(:,1)); %Number of buses

ng=length(Gbus); %Number of generators
PAR.ng = ng;
PAR.Gbus = Gbus;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LINEDATA = Line  from  to   R   X   B  tap 
% Run load flow calculations
[VOLT,ANG,LINFLW,P_LOSS,YBUS,Pmat,Qmat,Pinj,Qinj,iter]=lflow(BUSDATA,LINEDATA,flag_init,tole,maxiter);


PAR.YBUS = YBUS;
P0 = Pinj(:)+BUSDATA(:,5);
Q0= Qinj(:)+BUSDATA(:,6);

% Find the generated active and reactive powers
PG= Pinj(Gbus)'+BUSDATA(Gbus,5) ; % must be an ngx1 vector containing the generated active powers (see equation (A-14) and BUSDATA)
QG= Qinj(Gbus)'+BUSDATA(Gbus,6); % must be an ngx1 vector containing the generated reactive powers (see equation (A-15) and BUSDATA)

P0(Gbus) = PG - P0(Gbus);
Q0(Gbus) = QG - Q0(Gbus);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initial generator conditions, Sauer (1998), Ch 7.6
% Sauer, P., & Pai, M. (1998). Power System Dynamics and Stability. 
% Saddle River, NJ: Prentice hall

% STEP 1: Load-flow gives the voltages at generator buses
UG=VOLT(Gbus); 
ANGG=ANG(Gbus); 
UG = UG.*exp(1j*ANGG); % Complex voltage vector at generator buses

% Together with the injected power PG + j QG, we get the injected current
IG=conj((PG+j*QG)./(UG));

% STEP 2: Calculate rotor angle
XD = PAR.XD;
XDP = PAR.XDP;
if PAR.ns <= 3
    XQP = XDP; % Transient saliency is neglected
    XQ = XDP; % No rotor windings on q-axis   
else
    XQ = PAR.XQ;
    XQP = PAR.XQP;  
end
EQ=1j*XQ.*IG + UG; %
DELTA=angle(EQ);

% STEP 3: dq-values of current and generator bus voltage
Idq = IG.*exp(1j*(-DELTA+pi/2));
Id = real(Idq);
Iq = imag(Idq);
Udq = UG.*exp(1j*(-DELTA+pi/2));
Ud = real(Udq);
Uq = imag(Udq);

% STEP 4: Compute d-axis transient voltage 
EDP = (XQ-XQP).*Iq;
% (EDP =0 if no rotor winding on q-axis)

% STEP 5: Compute q-axis transient voltage
EQP = Uq + XDP.*Id;

% STEP 6: Compute field voltage at steady state
EF = EQP + (XD-XDP).*Id;

%% Assertions, compare generator to alternative calculations
if PAR.ns <= 3
    assert(norm( EQP - abs(EQ) ) < tole);
end

EDP_alt = Ud - XQP.*Iq;
assert(norm( EDP - EDP_alt ) < tole);
EF_alt=(XD.*EQP./XDP)-((XD-XDP).*abs(UG).*cos(DELTA-ANGG)./XDP);
assert(norm( EF - EF_alt ) < tole);

PE = EDP.*Id + EQP.*Iq + (XQP-XDP).*Id.*Iq;
QE = Id.*Uq - Iq.*Ud;
assert(norm( PG - PE ) < tole);
assert(norm( QG - QE ) < tole);

Iq_alt = (Ud-EDP)./XQP;
Id_alt = -(Uq-EQP)./XDP;
assert(norm( Iq - Iq_alt ) < tole);
assert(norm( Id - Id_alt ) < tole);


%% Save parameters
PAR.UREF = EF;
PAR.PM = PG;
PAR.EQP = EQP;
PAR.EDP = EDP;
PAR.PE0 = PE;
PAR.QE0 = QE;
PAR.ANG0 = ANG;
PAR.DELTA0 = DELTA;
PAR.U0 = VOLT;
PAR.PL0 = BUSDATA(:,5);
PAR.QL0 = BUSDATA(:,6);
PAR.P0 = P0;
PAR.Q0 = Q0;
PAR.LINEDATA = LINEDATA;
PAR.LINFLW = LINFLW;

OMEGA=zeros(ng,1);
PAR.x0 = [DELTA;OMEGA];
if PAR.ns >= 3
    PAR.x0 = [PAR.x0; EQP];
end
if PAR.ns >= 4
    PAR.x0 = [PAR.x0; EDP];
end

if PAR.ns == 6
% Flux equations for 6-th order machine
    F1D = EQP-(XDP-PAR.XL).*Id;
    F2Q = -EDP-(XQP-PAR.XL).*Iq;
    PAR.x0 = [PAR.x0; F1D; F2Q];    
end

PAR.nx = length(PAR.x0);

PAR.y0 = [PAR.ANG0; PAR.U0]; % two algebraic variables at each bus
PAR.ny = length(PAR.y0);

PAR.u_count = PAR.ng*2+PAR.nbus*2; % Number of standard control signals [PM; EF; PL; QL]

%% Governor
[PAR,opt] = load_GOV(PAR,opt);

%% Exciter Control
[PAR,opt] = load_EXC(PAR,opt);

%% Converter
[PAR,opt] = load_CONV(PAR,opt);

%% PODQ
[PAR,opt] = load_PODQ(PAR,opt);

%% PODP
[PAR,opt] = load_PODP(PAR,opt);

%% PMU
[PAR,opt] = load_PMU(PAR,opt);

%% Wind Turbine
[PAR,opt] = load_WIND(PAR,opt);

%% Include algbraic variables
PAR.xy0=[PAR.x0; PAR.y0];