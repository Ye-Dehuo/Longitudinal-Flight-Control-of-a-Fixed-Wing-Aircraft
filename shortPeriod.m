clc
clear all
close all

%% Modeling

% Typical parameters for a fixed-wing civil aircraft
W = 12224;
I_x = 1420.9; 
I_y = 4067.5;
I_z = 4786.0;
I_xz = 0;
S = 17.1;          
c = 1.74;  
b = 10.18; 
V_co = 122;

% Basic flight parameters
Ma = 0.158;                               
C_L_star = 0.41;
C_D_star = 0.05;
C_L_alpha = 4.44;
C_D_alpha = 0.33;
C_m_alpha = -0.683;
C_L_V = 0;                 
C_D_V = 0;
C_m_V = 0;
C_L_dot_alpha = 0;
C_z_dot_alpha = 0; 
C_m_dot_alpha = -4.36;
C_L_q = 3.80;
C_z_q = -3.80;
C_m_q = -9.96;
C_L_delta_e = 0.355;
C_z_delta_e = -0.355;
C_D_delta_e = 0;
C_m_delta_e = -0.923;
T_V = 0;
g = 9.81;
rho = 1.225;
gamma_star = 0;

% Additional parameters
m = W / g;
V_star = 340 * Ma;
q_star = rho * (V_star)^2/2;
K_alpha = C_L_alpha * q_star * S / W;
K_q = V_co / g;
C_star_ss = 2;

% Longitudinal dynamic derivatives
X_V = -((C_D_V + 2 * C_D_star) * q_star * S) / (m * V_star);                                 
X_alpha = -8.1231;                     
X_delta_e = -C_D_delta_e * (q_star * S) / m;                                    

Z_V = (C_L_V + 2 * C_L_star) * q_star * S / (m * V_star^2);                               
Z_alpha = ((C_D_star + C_L_alpha) * q_star * S) / (m * V_star);                                    
Z_delta_e = C_L_delta_e * (q_star * S) / (m * V_star);                        

M_bar_V = (C_m_V * q_star * S * c) / (V_star * I_y);                               
M_bar_alpha = C_m_alpha * (q_star * S * c) / I_y;
M_bar_dot_alpha = C_m_dot_alpha * (c / (2 * V_star)) * q_star * S * c / I_y;
M_bar_q = C_m_q * (c / (2 * V_star)) * q_star * S * c / I_y;
M_bar_delta_e = C_m_delta_e * (q_star * S * c) / I_y;

% Short period state-space model
A = [
    -Z_alpha, 1;
    M_bar_alpha - M_bar_dot_alpha * Z_alpha, M_bar_q + M_bar_dot_alpha
    ];

B = [
    -Z_delta_e;
    M_bar_delta_e - M_bar_dot_alpha * Z_delta_e
    ];

C = eye(2);% Assume output includes all state variables

D = [0;
     0]; % No direct control input effect on output

 % Start simulation
 sim('shortPeriodSimulink');
 
%% Root Locus Analysis

% Construct open-loop transfer function (without feedforward compensation)
num = [1.0000   43.8170  155.4970  112.6800];
den = [1.0000   16.0256   79.6081  203.0028  137.3250  0];
K_a = 0.35; % Amplifier parameter
K_G_star = K_a * 1.4836; % Forward path root locus gain
K_H_star = 1; % Feedback path root locus gain
K_star = K_G_star * K_H_star; % Root locus gain
sys_ol = K_star * tf(num, den); % Open-loop transfer function

% Plot root locus
figure('Name','Root locus')
rlocus(tf(num, den));

%% Stability Margin Analysis

% Plot Nyquist plot
figure('Name','Nyquist Plot')
nyquist(sys_ol);

% Plot Bode plot
figure('Name','Bode Plot')
bode(sys_ol);

% Gain and phase margins
[Gm,Pm,Wgm,Wpm] = margin(sys_ol); 

%% Handling Quality Analysis

sys_cl = tf([45.0270  192.6390  206.1238   58.5118], [1.0000   16.0268   80.1376  225.8221  218.1541   58.5412]); % Closed-loop transfer function

[Wn, Zeta, Pole] = damp(sys_cl); % Closed-loop system frequency, damping ratio, poles
Zero = zero(sys_cl); % Closed-loop system zeros

omega_n = Wn(3); % Underdamped system natural frequency (short-period mode natural frequency)
zeta = Zeta(3); % Underdamped system damping ratio (short-period mode damping ratio)
CAP = omega_n ^2 / ((V_star / g) * Z_alpha); % Control Anticipation Parameter

% Plot C* index graph
sim('shortPeriodSimulink');
C_star_0 = ones(size(t));
C_star_index = (Delta_C_star + C_star_0) / C_star_ss;
figure('name','C* Index')
plot(t, C_star_index, t, upperBound, t, lowerBound);
xlabel('$t/\mathrm{s}$','interpreter','latex');
ylabel('$C^*/C^*_\infty$','interpreter','latex');
grid on;

%% Time Domain Characteristics Analysis

% Rise time
tr_1 = (pi - acos(zeta))/(omega_n * sqrt(1 - zeta ^2));

% Peak time
tp_1 = pi / (omega_n * sqrt(1 - zeta ^2));

% Overshoot
sigma_1 = exp(-pi*zeta/sqrt(1-zeta^2)); 

% Settling time
ts_1 = 3.5/(zeta * omega_n);

% step(sys_cl);
