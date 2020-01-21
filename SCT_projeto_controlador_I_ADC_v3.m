clc;
clear;
close all;
opt = stepDataOptions('InputOffset',1,'StepAmplitude',0.5);

NA = 800;
T = 1/500;

%% Determinando equação do sistema de primeira ordem
%%%%%%%% Parâmetros medidos em osciloscópio
ts = 0.0044; % V
tau = ts/3;

%%%%%%%% Encontra a função de transferência
G1s = tf([1/tau],[1 1/tau])

%% Determinando equação do sistema de segunda ordem
%%%%%%%% Parâmetros medidos em osciloscópio

vp   = 1.61;   % V
vmin = 0.984;   % V
vinf = 1.51;   % V
tp   = 0.035; % s
ts_5 = 0.051; % s

%%%%%%%% Determinar Mp
delta_1 = vp - vinf;
delta_2 = vinf - vmin;
Mp = delta_1/delta_2;

%%%%%%%% Encontrar o zeta
syms zt;
eqn = exp(-pi*(zt/sqrt(1-zt^2))) == Mp;
zeta = vpasolve(eqn,zt);
zeta = double(zeta);

%%%%%%%% Encontrar o omega
syms w;
eqn = (pi/(w*sqrt(1-zeta^2))) == tp;
wn = vpasolve(eqn,w);
wn = double(wn);

%%%%%%%% Determinar a FT
num = [wn^2];
den = [1 2*zeta*wn wn^2];
G2s = tf(num,den)

%%%%%%%% Determinando a equação completa do sistema 
G1G2s = G1s*G2s;

%% Projeto [lugar das raizes]
Mp_proj = Mp/5;
Mp_ideal = Mp/2;

ts_5_proj = ts_5/2;
ts_5_ideal = ts_5/2;

%%%%%%%% Cálculo dos novos parâmetros
syms zt;
eqn = exp(-pi*(zt/sqrt(1-zt^2))) == Mp_proj;
zeta_proj = vpasolve(eqn,zt);
zeta_proj = double(zeta_proj);

wn_proj = 3/(zeta_proj*ts_5_proj);
wd_proj = wn_proj*sqrt(1 - zeta_proj^2);     % Freq. natural amortecida

%%%%%%%% Encontrar os pólos dominantes do sistema em malha fechada
mod_z = exp(-T*zeta_proj*wn_proj);
ang_z = T*wd_proj;

[z1_re,z1_im] = pol2cart(ang_z, mod_z); % pólo desejado em MF
z1 = complex(z1_re, z1_im);

%%%%%%%% Encontrando o equivalente discreto do sistema
Gz = c2d(G1G2s, T);
Gz = zpk(Gz)

%%%%%%%% Extraindo pólos e zeros da FT da planta
[n,d] = tfdata(Gz,'v');
z_gz = roots(n);
p_gz = roots(d);

%%%%%%%% Encontrar ângulos relativos ao pólo dominante de malha fechada
D1z = tf([1 -p_gz(1)], [1], T);
D2z = tf([1 -p_gz(2)], [1 -1], T);
Dz = minreal(D1z * D2z * Gz);
[n,d]=tfdata(Dz,'v');
ang_total = rad2deg(angle(polyval(n,z1)/polyval(d,z1)));
ang_res = 180 - ang_total;

%%%%%%%% Determinar o valor de beta
beta = z1_im/tan(deg2rad(-ang_res)) - z1_re;

%%%%%%%% Definição do ganho K
C1z = tf([1 -p_gz(1)], [1 beta], T);
C2z = tf([1 -p_gz(2)], [1 -1], T);
Cz = C1z * C2z;
Cz = zpk(minreal(Cz));

FTMA = Cz*Gz;
[num_ftma, den_ftma] = tfdata(FTMA,'v');
K = 1/abs(polyval(num_ftma,z1)/polyval(den_ftma,z1));
angulo_ftma = angle(polyval(num_ftma,z1)/polyval(den_ftma,z1));

%%%%%%%% Gc = K*(z+alpha)/(z+beta);
Cz = Cz*K

FTMA_sem_k = FTMA;
FTMA = zpk(Cz*Gz)

FTMF = minreal(FTMA/(1 + FTMA))

[num_ftma, den_ftma] = tfdata(FTMA + 1,'v');
ess = abs(polyval(den_ftma,1)/polyval(num_ftma,1));

% %%%%%%% Novo diagrama de polos e zeros
[n,d] = tfdata(FTMF,'v');
zeros = roots(n)
polos = roots(d)

%% Informações
%INFO = stepinfo(FTMF, 'SettlingTimeThreshold',0.05)
[y, t] = step(FTMF, opt);
INFO = stepinfo(y, t, 'SettlingTimeThreshold',0.05);

Mp_obtido = (INFO.SettlingMax - 1.5)/0.5;

% Informacoes do controlador
fprintf('Z1: %f + %fi\n', z1_re, z1_im);
fprintf('Angulo: %.2f°\n', rad2deg(angulo_ftma));
fprintf('Ganho K: %.4f\n', K);
fprintf('Erro em regime permanente: %.2f%%\n', ess);
fprintf('ts: %f [erro: %f%%]\n', INFO.SettlingTime, (INFO.SettlingTime - ts_5_ideal)/ts_5_ideal);
fprintf('Mp: %f [erro: %f%%]\n', Mp_obtido, (Mp_obtido - Mp_ideal)/Mp_ideal);
fprintf('tp: %f\n', INFO.PeakTime);

% Numerador e denominador pra equacao recursiva
[num_Cz, den_Cz]=tfdata(Cz,'v');
[num_Gz, den_Gz]=tfdata(Gz,'v');

%% Equação recursiva

s_1v5 = 1861;
s_1v  = 1241;
s_0v5 = 620;

r = [s_1v*ones(1, NA/2) s_1v5*ones(1, NA/2)];
c(1) = s_1v;
c(2) = s_1v;
c(3) = s_1v;
e(1) = 0;
e(2) = 0;
e(3) = 0;
x(1) = s_1v;
x(2) = s_1v;
x(3) = s_1v;
pwm(1) = 999*x(1)/4095;
pwm(2) = 999*x(2)/4095;
pwm(3) = 999*x(3)/4095;

for n=4:length(r)
    c(n) = num_Gz(2)*x(n-1) + num_Gz(3)*x(n-2) + num_Gz(4)*x(n-3) + (-1)*den_Gz(2)*c(n-1) + (-1)*den_Gz(3)*c(n-2) + (-1)*den_Gz(4)*c(n-3);
    e(n) = r(n) - c(n);
    x(n) = num_Cz(1)*e(n) + num_Cz(2)*e(n-1) + num_Cz(3)*e(n-2) + (-1)*den_Cz(2)*x(n-1) + (-1)*den_Cz(3)*x(n-2);
    
    if x(n)>4095
        x(n) = 4095;
    end
    
    if x(n)< 0
        x(n) = 0;
    end
    
    pwm(n) = 999*x(n)/4095;
end

for n=401:501
    c_n(n-(NA/2)) = c(n);
end

%% Graficos 
n = 0:100;
k = 0:NA-1;

figure(); step(G1s,opt), title ('Bloco de primeira ordem');
figure(); step(G2s,opt), title ('Bloco de segunda ordem');
figure(); step(G1G2s,opt), title ('Planta no domínio s');
figure(); rlocus(FTMA_sem_k), title ('Lugar das raízes de Gz*Cz'); 
figure(); zplane(zeros, polos), title ('Polos e zeros da FTMF');
figure(); step(FTMF, opt, 50*T); hold on;
          plot(n*T, 3.3*c_n/4095, 'r.'), title ('Resposta ao degrau');
figure(); plot(k*T, 3.3*x/4095, 'r.'), title ('Saída do controlador');
figure(); plot(k*T, pwm, 'r.'), title ('Valores de PWM');
figure(); plot(k*T, c, 'r.'), title ('Leituras do ADC');
figure(); plot(k*T, 3.3*e/4095, 'r.'), title ('Sinal de erro');
figure(); step(G1G2s,opt); hold on;
          step(d2c(FTMF), opt, 'r'), title ('Comparativo no domínio s');
figure(); step(Gz,opt); hold on;
          step(FTMF, opt, 'r'), title ('Comparativo no domínio z');

fprintf('Parâmetros: \n%.6f, %.6f, %.6f, %.6f, %.6f\n\n', num_Cz(1), num_Cz(2), num_Cz(3), (-1)*den_Cz(2), (-1)*den_Cz(3));
%close all;


