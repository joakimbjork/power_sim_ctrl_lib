function [E,V,W] = modes_sorted(G)
% W*A*V = E
G = ss(G);
[V,E] = eig(G.A);
E = diag(E);
n = length(E);
%  poorly damped swing modes
idx1 = find(abs(E)<40&abs(E)>1e-6);
E1 = E(idx1);
V1 = V(:,idx1); %eigenvectors
idx2 = 1:n;
idx2(idx1) = [];
E2 = E(idx2);
V2 = V(:,idx2);
damping = -real(E1)./abs(E1);
disp('Sorting modes by damping')
[damping,idx_sort] = sort(damping);
E1 = E1(idx_sort);
V1 = V1(:,idx_sort);
damping = -real(E2)./abs(E2);
[damping,idx_sort] = sort(damping);
E2 = E2(idx_sort);
V2 = V2(:,idx_sort);
E = [E1;E2];
V = [V1,V2];

freq = abs(E); %
damping = -real(E)./abs(E);
e = E(1); 
disp(['First mode: eig = ', num2str(e),...
      '; freq = ', num2str(abs(e)/(2*pi)),' Hz'...
      '; damping = ' num2str(-real(e)./abs(e)),' %']); 
 
W = inv(V); % W is matrix of left (row) eigenvectors 

end

