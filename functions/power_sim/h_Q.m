function [Qline]=h_Q(xy0,PAR)
nbus=PAR.nbus; %Number of buses
nx = PAR.nx;

%% States and algebraic variables
% x0=xy0(1:nx); %Current state variable values
y0=xy0(nx+1:nx+2*nbus); %Current algebraic variable values

ANG=y0(1:nbus);
VOLT=y0(nbus+1:2*nbus);



%% Reactive power output over transmission lines
Qline = sparse(PAR.nbus^2,1);
% Pline = sparse(PAR.nbus^2,1);
for from = 1:PAR.nbus
    for to = 1:PAR.nbus
        if from ~= to
            b12 = imag(PAR.YBUS(to,from));
            g12 = real(PAR.YBUS(to,from));

            Qline(from + (to-1)*PAR.nbus) = ...
                b12*(VOLT(from)^2 - VOLT(from)*VOLT(to)*cos(ANG(from)-ANG(to))) -...
                g12*VOLT(from)*VOLT(to)*sin(ANG(from)-ANG(to));

            % Pline(from + (to-1)*PAR.nbus) = ...
            %     g12*(VOLT(from)^2 - VOLT(from)*VOLT(to)*cos(ANG(from)-ANG(to))) +...
            %     b12*VOLT(from)*VOLT(to)*sin(ANG(from)-ANG(to));
        end
    end
end


%% Output
% Qline;

