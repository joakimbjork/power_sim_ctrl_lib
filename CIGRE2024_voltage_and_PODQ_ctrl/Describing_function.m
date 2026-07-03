opt_CIGRE2024
Gc = minreal(G/(1+G*K*H0*Hcom));
L = minreal(Kc_10*G*H0/(1+G*K*H0*Hcom));


x_osc = linspace(10^0.5,10^0.8,5000);
x = [logspace(-6,0.5,100),x_osc,logspace(0.8,3,1000)];
% D = 1.*exp(1j*x);

[fig1,~,co] = figureLatex(4,3.);
[tile, h1] = tiledLatex(1,1);

img = freqresp(L,1j*x);
img = squeeze(img);
plot(real(img),-imag(img),'Color',[1,1,1]*0.8); hold all
nline(1) = plot(real(img),imag(img),'Color',co(1,:));
% plot(real(img(1)),imag(img(1)),'o','Color',co(1,:)); hold all

xlim([-15,15])
ylim([-15,10])
% xline(-1)
xlabel('Real axis')
ylabel('Imag axis')


NA = @(A,d) 2/pi *(pi/2 - asin(d./A) - d./A.*sqrt(1-(d^2./A.^2))); % Describing function of dead-band
xA = linspace(1.01,10,1000)*db;
nline(2) = plot(-real(1./NA(xA,db)),-1*imag(1./NA(xA,db)),'r');
xA =1.285
nline(3) = plot(-real(1./NA(xA*db,db)),0,'+r','markersize',10);

% x_osc = linspace(10^0.595,10^0.60,100);
% img_osc = freqresp(Gc*Kc_10,1j*x_osc);
% img_osc = squeeze(img_osc);
% plot(real(img_osc),imag(img_osc),'Color',co(2,:)); hold all
% dL = arrowsLatex(img_osc,1,co(1,:),4,[-0.1,0.1]);
% grid on

annotation('textarrow',[0.5,0.45]-0.05,[0.4,0.45]-0.1,'String','\omega\rightarrow\infty')

annotation('textarrow',[0.2,0.3]+0.1,[0,0]+0.75,'String','A\rightarrow\infty ')

l=legendLatex(gcf,{'$G_c(j\omega)H(j\omega)K_c(j\omega)$','$-1/N(A)$',['$-1/N(A)$, $A ={}$',num2str(xA) ,'$\times \mathrm{db}$']},nline);
set(l,'location','northeast')

saveLatex(fig1,'describing_func_nyquist',fig_save_opt)

%% Bode diagram
opt_CIGRE2024

[fig2,name,co] = figureLatex(4,3.5);
bodeopt = struct();
f_comp = 1;
% bodeopt.FreqUnits = 'Hz';
omega_lim = log10( [1e-2,2*pi*1e2] );

% bodeopt.omega = logspace(omega_lim(1),omega_lim(2),10000);
bodeopt.omega_lim = [1e-3,0.85e2]*2*pi ;

bodeopt.color = co(1,:);
bodeopt = bodeLatex(Gc*H0*Kc_10,bodeopt);
bodeopt.color = co(2,:);
bodeopt.linestyle = '-';
bodeopt = bodeLatex(Gc,bodeopt);
nexttile(1)
set(gca, 'Children', flipud(get(gca, 'Children')) )

bodeopt.color = co(3,:);
bodeopt.linestyle = '--';
bodeopt = bodeLatex(G,bodeopt);
nexttile(1)
set(gca, 'Children', flipud(get(gca, 'Children')) )

nexttile(1)
xticklabels([])
yline(1)
ylim([10^-3,10^2])
nexttile(2)
ylim([-270,90])
yline(-180)
bode_draw_lines_top([0.25*2*pi,1*2*pi]/f_comp,{'$\omega_1$','$\omega_2$'})


l=legendLatex(gcf,{'$G_c(j\omega)H(j\omega)K_c(j\omega)$',...
    '$G_c(j\omega)$',...
    '$G(j\omega)$'});
set(l,'location','southwest')
saveLatex(fig2,'bode_PODQ_and_voltage_cl',fig_save_opt)

