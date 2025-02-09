clc
clear all
close all

%% 建模

% 典型固定翼民机参数
W = 12224;
I_x = 1420.9; 
I_y = 4067.5;
I_z = 4786.0;
I_xz = 0;
S = 17.1;          
c = 1.74;  
b = 10.18; 
V_co = 122;

% 基础飞行参数
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

% 其它参数
m = W / g;
V_star = 340 * Ma;
q_star = rho * (V_star)^2/2;
K_alpha = C_L_alpha*q_star*S/W;
K_q = V_co/g;
C_star_ss = 2;

% 纵向动力学导数
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

% 短周期状态空间模型
A = [
    -Z_alpha, 1;
    M_bar_alpha - M_bar_dot_alpha * Z_alpha, M_bar_q + M_bar_dot_alpha
    ];

B = [
    -Z_delta_e;
    M_bar_delta_e - M_bar_dot_alpha * Z_delta_e
    ];

C = eye(2);% 假设输出为所有状态变量

D = [0;
     0]; % 没有直接的控制输入到输出的影响

 % 开始仿真
 sim('shortPeriodSimulink');
 
%% 根轨迹分析

% 建立开环传递函数（无前馈补偿）
num = [1.0000   43.8170  155.4970  112.6800];
den = [1.0000   16.0256   79.6081  203.0028  137.3250  0];
K_a = 0.35; % 放大器参数
K_G_star = K_a * 1.4836; % 前向通道根轨迹增益
K_H_star = 1; % 反馈通道根轨迹增益
K_star = K_G_star * K_H_star; % 根轨迹增益
sys_ol = K_star*tf(num, den); % 开环传递函数

% 绘制根轨迹
figure('Name','Root locus')
rlocus(tf(num, den));

%% 稳定裕度分析

% 绘制奈奎斯特图
figure('Name','Nyguist Plot')
nyquist(sys_ol);

% 绘制Bode图
figure('Name','Bode Plot')
bode(sys_ol);

% 幅值裕度 相角裕度
[Gm,Pm,Wgm,Wpm] = margin(sys_ol); 

%% 操纵品质分析

sys_cl = tf([45.0270  192.6390  206.1238   58.5118], [1.0000   16.0268   80.1376  225.8221  218.1541   58.5412]); % 闭环传递函数

[Wn, Zeta, Pole] = damp(sys_cl); % 闭环系统频率，阻尼比，极点
Zero = zero(sys_cl); % 闭环系统零点

omega_n = Wn(3); % 欠阻尼系统自然频率（受控后短周期模态自然频率）
zeta = Zeta(3); % 欠阻尼系统阻尼比（受控后短周期模态阻尼比）
CAP = omega_n ^2 / ((V_star / g) * Z_alpha); % 操纵期望参数

% 绘制C*指标图
sim('shortPeriodSimulink');
C_star_0 = ones(size(t));
C_star_index = (Delta_C_star + C_star_0) / C_star_ss;
figure('name','C*指标')
plot(t, C_star_index, t, upperBound, t, lowerBound);
xlabel('$t/\mathrm{s}$','interpreter','latex');
ylabel('$C^*/C^*_\infty$','interpreter','latex');
grid on;

%% 时域特性分析

% 上升时间
tr_1 = (pi - acos(zeta))/(omega_n * sqrt(1 - zeta ^2));

% 峰值时间
tp_1 = pi / (omega_n * sqrt(1 - zeta ^2));

% 超调量
sigma_1 = exp(-pi*zeta/sqrt(1-zeta^2)); 

% 调节时间
ts_1 = 3.5/(zeta * omega_n);

% step(sys_cl);
