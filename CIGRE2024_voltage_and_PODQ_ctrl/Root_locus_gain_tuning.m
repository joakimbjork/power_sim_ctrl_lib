opt_CIGRE2024

F_PD0 = K_POD_0 * H0;
F_PD = pade(K_POD * H,10);
[fig,names,co] = figureLatex(4,3.5);
tile = tiledLatex(1,1);

% [R,K] = rlocus(L,K);
% rlocus(L)
% xlim([-0.01,0.01])
% ylim([-0.1,8])

K = [0,logspace(-0.3,3,57),10^6];
% K = linspace(0,1000,1000);
c_idx = 1;
for i = 1:2
    if i == 1
        L = G*K_POD_0* pade(H,10);
    elseif i == 2
        L = G*K_POD_0 * H0;
    end
    [R,K] = rlocus(L,K); 
    
    w0 = 2*pi/opt.period_time;
    
    wR = imag(R(:,1)); % Imaginary part of all modes (K=0);
    wR_dist = abs(w0-wR);
    [w_mode,idx_mode] = min(wR_dist); 
    R_mode = R(idx_mode,:);
    damping = -real(R_mode)./abs(R_mode);
    [d_max,idx] = max(damping);
    Kopt = K(idx);

    r_idx = 1:length(K);
%     threshold = 3e-4; %can be any duration: minutes(30), hours(12) days(2), etc....
%     r_idx = [true, abs(diff(abs(R_mode)))>threshold];
%     r_idx(end) = true;
    
%     R_mode_red = R_mode(r_idx);
    % OR
    % dates(rm) = [];  % to keep variable name

    if i == 1
        names{3*(c_idx-1)+1} = ['Closed-loop pole, $\tau={}$75\,ms'];
        plot(real(R_mode(r_idx)),imag(R_mode(r_idx)),'color',co(c_idx,:)); hold all
    else
        names{3*(c_idx-1)+1} = ['Closed-loop pole, $\tau={}$0\,ms'];
        plot(real(R_mode(r_idx)),imag(R_mode(r_idx)),'--','color',co(c_idx,:)); hold all
    end
    plot(real(R_mode(idx)),imag(R_mode(idx)),'x','color',co(c_idx,:)); hold all  
    names{3*(c_idx-1)+2} = [num2str(round(100*d_max,2)),'\% damping $@k={}$', num2str(round(K(idx))),'\,p.u.'];
    
    K_a = 8;
    [K_a,idx_a] = min(abs(K-K_a)) 
    plot(real(R_mode(idx_a)),imag(R_mode(idx_a)),'o','color',co(c_idx,:)); hold all  
    d_a = -real(R_mode(idx_a))/abs(R_mode(idx_a));
    names{3*(c_idx-1)+3} = [num2str(round(100*d_a,2)),'\% damping $@k={}$', num2str(round(K(idx_a))),'\,p.u.'];

    c_idx = c_idx+1;
end


ylabel('Imaginary [rad/s]')
xlabel('Real [rad/s]')
xlims = xlim;
xlim(xlims+[-0.001,0.001])
grid on
l = legendLatex(fig,names); set(l,'location','northwest')
saveLatex(fig,'rlocus_PD',fig_save_opt)