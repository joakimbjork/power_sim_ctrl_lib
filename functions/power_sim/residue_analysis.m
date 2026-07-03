function [mode] = residue_analysis(G)
% [mode] = residue_analysis(G)
[m,n] = size(G);
assert(m==1)
assert(m==n)
G = ss(G);
A = G.A;
B = G.B(:,1);
C = G.C(1,:);

[V,E] = eig(A); % W'*A*V = E  
W = inv(V)';

[E,idx] = sort(E);
% C = Cd*V(:,idx);
V = V(:,idx);
W = W(:,idx);

idx = find(imag(E)>0); % Only care about non-negative imaginary parts
E = E(idx);
% C = C(:,idx);
V = V(:,idx);
W = W(:,idx);

idx = find(abs(E)>0.1*2*pi);
E = E(idx);
% C = C(:,idx);
V = V(:,idx);
W = W(:,idx);


damping = -real(E)./abs(E);
[~,idx] = sort(damping);
E = E(idx);
% C= C(:,idx);
V = V(:,idx);
W = W(:,idx);

mode = {};
mode.e = E;
mode.f = abs(E)/(2*pi);
mode.d = -real(E)./abs(E);
mode.obs = C*V;
mode.ctrb = (W'*B).';
mode.residue = mode.obs.*mode.ctrb;
mode.r_abs = abs(mode.residue);
r_ang = angle(mode.residue);
r_ang = sign(r_ang).*pi-r_ang;
mode.r_ang = r_ang*180/pi;
mode.N = length(E);



