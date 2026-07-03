function [mode] = modal_shape(Ad,Cd,Ts,f_track)
% A = logm(Ad)/Ts; % Continious state matrix (only care about eigenvalues)
tole = 1e-4;

[V,Ed] = eig(Ad); % W'*A*V = E  
if any(Ts)
    E = log(diag(Ed))/Ts; % Eigenvalues of continious system
else
    E = diag(Ed);
end
[E,idx] = sort(E);
C = Cd*V(:,idx);

idx = find(imag(E)>=0); % Only care about non-negative imaginary parts
E = E(idx);
C = C(:,idx);

[E_all,idx] = sort(real(E),'descend'); % Save all egienvalues, largest real part first
C_all = C(:,idx);

idx = find(imag(E)>0); % Only care about positive imaginary parts
E = E(idx);
C = C(:,idx);

idx = find(abs(E)>tole );
E = E(idx);
C = C(:,idx);

damping = -real(E)./abs(E);
[~,idx] = sort(damping);
E = E(idx);
C= C(:,idx);

for i_m = 1:size(C,2)
    v = C(:,i_m);
    [~,idx] = max(abs(v));
    ang = angle(v(idx));
    C(:,i_m) = v*exp(-1j*ang)/norm(v); % Normalize and rotate vector 
end

f = abs(E)/(2*pi);

if exist('f_track','VAR')
    found = false;
    for i = 1:length(f);
        if abs(f(i)-f_track(1))<abs(f_track(2))
            found = true;
            break
        end
    end
    if found
        idx = i;
    else
        idx = 1;
    end
else
    idx = 1:length(f);
end

mode = {};
mode.e = E(idx);
mode.v = C(:,idx);
mode.f = abs(E(idx))/(2*pi);
mode.d = -real(E(idx))./abs(E(idx));
         mode.observed = true(size(E(idx)));
mode.N = length(E(idx));

mode.e_all = E_all;
mode.v_all = C_all;

end

