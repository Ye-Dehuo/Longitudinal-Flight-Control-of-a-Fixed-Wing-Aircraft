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

% 状态空间模型
A = [
    X_V, X_alpha + g*cos(gamma_star), 0, -g*cos(gamma_star);
    -Z_V, -Z_alpha + (g*sin(gamma_star)/V_star), 1, -(g*sin(gamma_star)/V_star);
    M_bar_V - M_bar_dot_alpha * Z_V, M_bar_alpha - M_bar_dot_alpha * (Z_alpha - (g*sin(gamma_star)/V_star)), M_bar_q + M_bar_dot_alpha, -M_bar_dot_alpha * (g*sin(gamma_star)/V_star);
    0, 0, 1, 0
];

B = [
    X_delta_e;
    -Z_delta_e;
    M_bar_delta_e - M_bar_dot_alpha * Z_delta_e;
    0
];

C = eye(4); % 假设输出为所有状态变量
D = zeros(4, 1);  % 没有直接的控制输入到输出的影响

% 创建状态空间对象
sys = ss(A, B, C, D);

%% 模态分析

% 求解特征根
eigValues = eig(A); % 系统特征根
eta_sp = real(eigValues(1)); % 短周期模态对应的特征根实部
eta_p = real(eigValues(3)); % 长周期模态对应的特征根实部

% 短周期模态（取近似模型）
omega_n_sp = sqrt(-(M_bar_alpha+M_bar_q*Z_alpha)); % 自然频率
zeta_sp = -(M_bar_q+M_bar_dot_alpha-Z_alpha)/(2*omega_n_sp); % 阻尼比
t_12 = -log(2)/eta_sp; % 半衰期
N_12 = log(2)*sqrt(1-zeta_sp^2)/(2*pi*zeta_sp); % 半衰期内振荡次数
omega_sp = omega_n_sp*sqrt(1-zeta_sp^2); % 阻尼振荡频率
T_sp = 2*pi/omega_sp; % 周期

% 长周期模态
omega_p = imag(eigValues(3)); % 阻尼振荡频率
T_p = 2*pi/omega_p; % 周期
omega_n_p = sqrt(eta_p^2+omega_p^2); % 自然频率
zeta_p = -eta_p/omega_n_p; % 阻尼比
t_2 = -log(2)/eta_p; % 倍幅时
