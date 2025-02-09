# 固定翼飞机纵向飞行控制
## 概述
+ 飞机模态和飞行品质分析主要基于全周期动力学模型，源码可见 `allPeriod.m`<br><br>
+ 飞行控制、根轨迹、稳定裕度、操纵品质与时域特性分析主要基于短周期模型，源码可见 `shortPeriod.m`，Simulink 模型可见 `shortPeriodSimulink.slx`
## 1. 建模

### 参数确定

#### 1. 典型固定翼民机参数

飞行器重量 $W = 12224 \ N​$<br>

惯矩 $I_x=1420.9 \mathrm{~kg} \cdot \mathrm{~m}^2, I_y=4067.5 \mathrm{~kg} \cdot \mathrm{~m}^2, I_z=4786.0 \mathrm{~kg} \cdot \mathrm{~m}^2, I_{x z}=0 \mathrm{~kg} \cdot \mathrm{~m}^2$<br>

机翼面积 $S=17.1 \mathrm{~m}^2$, 平均气动弦长 $c=1.74 \mathrm{~m}$, 展长 $b=10.18 \mathrm{~m}$

#### 2. 基础飞行参数

飞机在海平面以 $M a=0.158$ 飞行, $C_{L *}=0.41, C_{D *}=0.05$<br>

迎角导数 $C_{L \alpha}=4.44, C_{D \alpha}=0.33, C_{m \alpha}=-0.683​$<br>

速度导数 $C_{L V}=0.0, C_{D V}=0.0, C_{m V}=0.0$<br>

动导数 $C_{L \dot{\alpha}}=C_{z \dot{\alpha}}=0.0, C_{m \dot{\alpha}}=-4.36, C_{L q}=-C_{z q}=3.80, C_{m q}=-9.96$<br>

操纵导数 $C_{L \delta_e}=-C_{z \delta_e}=0.355, C_{D \delta_e}=0.0, C_{m \delta_e}=-0.923$<br>

不计推力随速度的变化，则 $T_V = 0$<br>

重力加速度 $g = 9.81 \ m/s^2$<br>

大气密度 $\rho = 1.225 \ kg/m^3$

#### 3. 其它参数

质量 $m = \frac{W}{g}$<br>

基准速度 $V_* = Ma \times 340 $<br>

基准动压 $q_* = \frac{\rho {V_*}^2}{2}$<br>

假设风速为0，飞机无侧滑水平直线飞行 $\gamma_* = 0$ ，且始终保持在同一高度<br>

### 模型建立

基于纵向小扰动方程的简化形式进行建模，且不计扰动运动中高度变化引起的外力和力矩的影响，并仅考虑通过升降舵对飞机进行纵向飞行控制，模型矩阵形式建立为<br>
```math
\frac{\mathrm{d}}{\mathrm{d} t}\left[\begin{array}{c}\Delta V \\ \Delta \alpha \\ \Delta q \\ \Delta \theta\end{array}\right]=\left[\begin{array}{cccc}X_V & X_\alpha+g & 0 & -g \\ -Z_V & -Z_\alpha & 1 & 0 \\ \bar{M}_ V-\bar{M}_ {\dot{\alpha}} Z_V & \bar{M}_ \alpha-\bar{M}_ {\dot{\alpha}} Z_\alpha & \bar{M}_ q+\bar{M}_ {\dot{\alpha}} & 0 \\ 0 & 0 & 1 & 0\end{array}\right]     \left[\begin{array}{c}\Delta V \\ \Delta \alpha \\ \Delta q \\ \Delta \theta\end{array}\right]+\left[\begin{array}{c}X_{\delta_{\mathrm{e}}} \\ -Z_{\delta_{\mathrm{e}}} \\ \bar{M}_ {\delta_{\mathrm{e}}}-\bar{M}_ {\dot{\alpha}} Z_{\delta_{\mathrm{e}}} \\ 0 \end{array}\right]\Delta \delta_{\mathrm{e}}
```
<br>
涉及11个纵向动力学导数，分别为<br><br>

$X_V = \frac{T_V \cos \left( \alpha_* + \varphi_T \right) }{m} - \frac{\left( C_{D V} + 2 C_{D*} \right)}{m V*}$<br>

$X_\alpha=\frac{-T* \sin \left(\alpha*+\varphi_T\right)-C D \alpha q* S}{m}$<br>

$X_{\delta_{\mathrm{e}}}=-C_{D \delta_{\mathrm{e}}} \frac{q * S}{m} $<br>

$Z_V=\frac{T_V \sin \left(\alpha*+\varphi\right)}{m V*}+\frac{\left(C_{L V}+2 C_{L*}\right) q* S}{m V*^2}$<br>

$Z_\alpha=\frac{\left(C_{D*}+C_{L \alpha}\right) q* S}{m V*}=\frac{T* \cos \left(\alpha* + \varphi_T\right)+C_{I \alpha} q* S}{m V*}$<br>

$Z_{\delta_{\mathrm{e}}}=C_{L \delta_{\mathrm{e}}} \frac{q* S}{m V*}$<br>

$\bar{M}_ V = \frac{ (C_{mV} + 2 C_{m*}) q * Sc}{2}$<br>

$\bar{M}_ \alpha=C_{m \alpha} \frac{q* S c}{I_y}$<br>

$\bar{M}_ {\dot{\alpha}}=C_{m \dot{\alpha}}\left(\frac{c}{2 V*}\right) \frac{q* S c}{I_y} $<br>

$\bar{M}_ q=C_{m q}\left(\frac{c}{2 V*}\right) \frac{q* S c}{I_y}$<br>

$\bar{M}_ {\delta_{\mathrm{e}}}=C_{m \delta_e} \frac{q* S c}{I_y} $

## 2. 模态与飞行品质分析

### 模态分析

#### 1. 短周期模态（取近似模型）

自然频率 $\omega_{n, \ sp }=\sqrt{-\left(\bar{M}_ a+\bar{M}_ q Z_\alpha\right)} = 3.6138$<br>

阻尼比 $\zeta_{\mathrm{sp}}=-\frac{\bar{M}_ q+\bar{M}_ {\dot{\alpha}}-Z_\alpha}{2 \omega_{\mathrm{n} . \mathrm{sp}}} = 0.6954$<br>

半衰期 $t_{1 / 2}=-\frac{\ln 2}{\eta_{sp}} = 0.275$<br>

半衰期内振荡次数  $N_{1 / 2}=\frac{\ln 2 \sqrt{1-\xi^2_{sp}}}{2 \pi \xi_{sp}} = 0.114$<br>

阻尼振荡频率 $\omega_{sp} = \omega_{n, \ sp}\sqrt{1-{\xi^2_{sp}}} = 2.597$<br>

周期 $T_{sp} = \frac{2\pi}{\omega_{n, \ sp}} = 2.4194$

#### 2. 长周期模态

阻尼振荡频率 $\omega_{p} = Im(\lambda) = 0.213$<br>

周期 $T_p = \frac{2\pi}{\omega_{n, \ p}} = 29.4923$<br>

自然频率 $\omega_{\mathrm{n} , \ \mathrm{p}}= \sqrt{\eta^2_p + \omega_p^2} = 0.2137$<br>

阻尼比 $\zeta_{\mathrm{p}}= \frac{-\eta_p}{w_n, \ p} = 0.0798$<br>

倍幅时 $t_{2}=-\frac{\ln 2}{\eta_p} = 40.6338$

### 飞行品质分析

#### 1. 短周期模态

 $0.35 < \zeta_{\mathrm{sp}} = 0.6954 < 1.3$<br>

短周期阻尼比满足要求

#### 2. 长周期模态

(1) 阻尼比<br>

$\zeta_p = 0.0798 \ge 0.04$<br>

长周期阻尼比满足要求 

(2) 倍幅时<br>

 $t_2 = 40.6338 < 55$<br>

倍幅时不满足要求

## 3. $C^*$控制律设计

$C^* = n_n+\frac{V_{\mathrm{co}}}{g} q$<br>

其中， $V_{co}$ 为穿越速度，取值范围一般为 $120 \sim 132 m/s$，此处取 $V_{co} = 122 m/s$<br>

控制律针对 $\Delta C^*$ 进行设计，定义跟踪误差<br>

 $e(t) = \Delta C^{**} - \Delta C^*$<br>

基于 PI 控制方法设计 $C^*$ 控制律如下<br>

$u(t) = {K_P e(t) + K_I \int e(t) dt}$<br>

舵机模型设为<br>

$G_\delta(s) = \frac{-1}{0.1s+1}$

## 4. 控制系统建立与分析

### 控制系统建立

对于 $C^*$ 指标的控制通常要求在较短时间内完成，于是将动力学模型简化为短周期模型，并仅考虑通过升降舵对飞机进行控制<br>
```math
\left[\begin{array}{c}\Delta \dot{\alpha} \\ \Delta \dot{q}\end{array}\right]=\left[\begin{array}{cc}-Z_\alpha & 1 \\ \bar{M}_\alpha-\bar{M}_{\dot{\alpha}} Z_\alpha & \bar{M}_q+ \bar{M}_{\dot{\alpha}}\end{array}\right]\left[\begin{array}{c}\Delta \alpha \\ \Delta q\end{array}\right] + \left[\begin{array}{c}-Z_{\delta_e} \\ \bar{M}_{\delta_e}-\bar{M}_{\dot{\alpha}} Z_{\delta_e}\end{array}\right] \Delta \delta_e 
```
<br>
法向过载改变量与迎角改变量间存在如下关系<br><br>

$\Delta n_n = \frac{C_{L\alpha} q S \Delta \alpha}{W}$<br>

闭环系统输出为<br>

$\Delta C^* = \Delta n_n +\frac{V_{\mathrm{co}}}{g} \Delta q$<br>

控制系统由增稳系统 S.A.S 与控制增强系统 C.A.S构成

+ S.A.S<br>

  纵向增稳器采用法向过载作为反馈，放大器增益为 $K_{n1}​$ <br>

  俯仰阻尼器中，放大器增益为 $K_{q1}$，清洗网络时间常数设为 $1$<br>

+ C.A.S<br>

  指令前馈补偿器中，放大器增益为 $K_{ff}$。前馈补偿器可通过增加相位来调节系统中的一个零点，同时不会改变系统的极点<br>

  控制放大器增益为 $K_a$<br>

在 Simulink 中建立如下控制系统<br>

![alt](/img/shortPeriodSimulink.jpg)

控制系统输出 $\Delta C^*$ 如下图所示<br>

![alt](/img/Delta_C_star.jpg)

 $\Delta q$ 如下图所示

![alt](/img/Delta_q.jpg)

### 根轨迹分析

暂时忽略前馈补偿器，取  $K_{n1} = 0.05, K_{q1} = 0.01, K_P = 0.001, K_I = 0.04$<br>

由分析可得，系统开环传递函数为<br>

$G_{ol}(s) = K^* \frac{s^3 + 43.817s^2 + 155.497s + 112.68}{s^5 + 16.0256s^4 + 79.6081s^3 + 203.0028s^2 + 137.325s}$<br>

$K^* = K_G^*  K_H^* = 1.4836 K_a K_H^* = 1.4836 K_a$<br>

其中，$K^*$ 为开环根轨迹增益，$K_G^* $ 为前向通路根轨迹增益，$K_H^* $ 为反馈通路根轨迹增益，$K_a$ 为控制放大器增益<br>

可绘制出系统根轨迹图如下<br>

![alt](/img/rlocus1.jpg)

![alt](/img/rlocus2.jpg)

![alt](/img/rlocus3.jpg)

由根轨迹图可知，当$0< K^* < 12.5$时，根轨迹均位于左半 $s$ 平面，此时系统稳定

### 稳定裕度分析

Nyquist 图如下所示<br>

![alt](/img/nyguist.jpg)

Bode 图如下所示<br>

![alt](/img/bode.jpg)

幅值裕度为 $24.8$<br>

相角裕度为 $87.5°$

### 操纵品质分析

(1) $CAP$ 指标

取 $K_a = 0.35, K_{ff} = 0.03$，，可求得此时闭环传递函数为<br>

$G_{cl}(s) = \frac{45.03s^3 + 192.6s^2 + 206.1s + 58.51}{s^5 + 16.03s^4 + 80.14s^3 + 225.8s^2 + 218.2s + 58.54}$<br>

并得到此时的短周期模态自然频率 $\omega_{n, sp} = 3.67$，短周期模态阻尼比 $\zeta_{n, sp} = 0.61$<br>

操纵期望参数 $CAP = \frac{\omega_{\mathrm{n}, \mathrm{sp}}^2}{(V \star / g) Z_\alpha} = 1.21$<br>

该项目主要对飞机巡航阶段的控制进行研究，因此飞机处于 B 种飞行阶段，相应的短周期操纵品质要求如下图所示<br>

![alt](/img/CAP_CATB.jpg)

从图中可以看出，该飞机在巡航阶段的操纵期望参数满足 1 级飞行品质要求<br>

(2) $C^*$ 指标

输入为 $1g$ 时 $C^*$ 指标如下图所示，可以看出该飞机 $C^ *$ 指标满足包线约束<br>

![alt](/img/C_star_index.jpg)

### 时域特性分析

（1）动态性能指标<br>

闭环极点如下图所示<br>

![alt](/img/closed_loop_poles_zeros.jpg)

系统共有 5个闭环极点，包括 3 个实数极点与 1 对共轭极点<br>

$P_1 (-0.4405, 0)$<br>

$P_2 (-0.9783, 0) $<br>

$P_3 ( -2.254, 2.893)$<br>

$P_4 ( -2.254, -2.893)$<br>

$P_5 (-10.1, 0)$<br>

闭环零点共有 3 个<br>

$Z_1 (-2.82, 0)$<br>

$Z_2 (-1, 0)$<br>

$Z_3 (-0.46, 0)$<br>

位于虚轴附近的极点 $P_1$ 与零点 $Z_3$ ，极点 $P_2$ 与 零点$Z_2$ 分别距离过近，对于系统响应的影响会互相削弱。极点 $P_5$ 距 $P_3, P_4$ 相对较远。因此，系统响应主要由剩下共轭极点对 $P_3, P_4$ 与零点 $Z_1$ 确定<br>

共轭极点 $P_3， P_4$ 对应欠阻尼系统，自然频率 $w_{n} = 3.67 $ ，阻尼比 $\zeta = 0.61$<br>

闭环零点 $Z_1$ 对系统动态性能的影响主要为减小峰值时间，使系统响应速度加快， 但会使超调量增大<br>

在输入为阶跃信号下<br>

+ 上升时间

  欠阻尼系统上升时间 $t_{r}=\frac{\pi-arccos\zeta}{\omega_d} = 0.77 \ s$<br>

  系统实际上升时间 $t_r = 0.31 \ s$<br>

+ 峰值时间

  欠阻尼系统峰值时间 $ t_{p2} = 1.09 \ s$<br>

  系统实际峰值时间 $t_p = 0.74 \ s$<br>

+ 超调量

  欠阻尼系统超调量 $\sigma_2 \%=\mathrm{e}^{-\pi \zeta / \sqrt{1-\zeta^2}} \times 100 \% = 8.7\%$<br>

  系统实际超调量 $\sigma \%= 18 \%$<br>

+ 调节时间

  欠阻尼系统调节时间 $t_{s2} = \frac{3.5}{\zeta \omega_n} = 1.55 \ s$<br>

  系统实际调节时间 $t_s = 2.45 \ s$<br>

（2）稳态性能指标<br>

开环系统在坐标原点有一个极点， 所以系统属 I 型系统，因而系统可以无误差跟踪阶跃信号

