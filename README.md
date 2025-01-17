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

$G_\delta(s) = \frac{-1}{5s+1}$

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

在 Simulink 中建立如下控制系统<br>

![alt](/img/shortPeriodSimulink.png)

控制系统输出 $\Delta C^*$ 如下图所示<br>

![alt](/img/Delta_C_star.png)

### 根轨迹分析

分析可得，系统开环传递函数为<br>

$G_o(s) = K^* \frac{s^2 + 3.017s + 0.563}{s^4 +5.226s^3 + 14.065s^2 + 2.612s}$<br>

其中， $K^*$ 为开环根轨迹增益<br>

可绘制出系统根轨迹图如下<br>

![alt](/img/rlocus.jpg)

由根轨迹图可知，当 $K^* > 0$ 时，根轨迹均位于左半 $s$ 平面，因此系统对所有 $K^*$ 增益都是稳定的

### 稳定裕度分析

Nyquist 图如下所示<br>

![alt](/img/nyguist.jpg)

Bode 图如下所示<br>

![alt](/img/bode.jpg)

幅值裕度将裕度趋于一个很大的值<br>

相角裕度为 $50.68°$

### 操纵品质分析

(1) $CAP$ 指标

取 $K_P = 0.5, K_I = 0.1$，可得开环根轨迹增益 $K^* = 14.84$，由根轨迹图可确定此时的闭环极点，并得到此时的短周期模态自然频率 $\omega_{n, sp} = 4.686$，短周期模态阻尼比 $\zeta_{n, sp} = 0.333$<br>

操纵期望参数 $CAP = \frac{\omega_{\mathrm{n}, \mathrm{sp}}^2}{(V \star / g) Z_\alpha} = 1.978$<br>

该项目主要对飞机巡航阶段的控制进行研究，因此飞机处于 B 种飞行阶段，相应的短周期操纵品质要求如下图所示<br>

![alt](/img/CAP_CAT_B.jpg) 

从图中可以看出，该飞机在巡航阶段的操纵期望参数满足 1 级飞行品质要求<br>

(2) $C^*$ 指标

输入为 $1g$ 时 $C^*$ 指标如下图所示，可以看出该飞机 $C^ *$ 指标满足包线约束<br>

![alt](/img/C_star_index.png)

### 时域特性分析

（1）动态性能指标<br>

闭环极点如下图所示<br>

![alt](/img/closed_loop_poles_zeros.jpg)

系统共有4个闭环极点，包括2个实数极点与1对共轭极点<br>

$(0.1998, 0)$<br>

$(-1.904, 0) $<br>

$(-1.561, 4.418)$<br>

$(-1.561, -4.418)$<br>

位于虚轴附近的闭环极点 $(0.1998, 0)$ 与闭环零点 $(2, 0)$ 距离过近，两者对于系统响应的影响会互相削弱。因此，系统响应主要由剩下的1个实数极点与1对共轭极点确定<br>

设实数极点为 $p1 (-1.904, 0)$，对应一阶系统，时间常数 $T = \frac{1}{w} $<br>

设共轭极点为 $p2(-1.561, 4.418) \ \ \ p3(-1.561, -4.418)$ ，对应欠阻尼系统，自然频率 $w_n = 4.686$，阻尼比 $\zeta = 0.333$<br>

在输入为阶跃信号下<br>

+ 上升时间

  一阶系统上升时间 $t_{r1} = 2.2 T = 1.55 \ s$<br>

  欠阻尼系统上升时间 $t_{r2}=\frac{\pi-arccos\zeta}{\omega_d} = 0.43 \ s$<br>

  系统实际上升时间 $t_r = 0.38 \ s$

+ 峰值时间

  一阶系统不具有峰值时间<br>

  欠阻尼系统峰值时间 $t_{p2} = 0.71 \ s$<br>

  系统实际峰值时间 $t_p = 0.80 \ s$

+ 超调量

  一阶系统不具有超调量<br>

  欠阻尼系统超调量 $\sigma_2 \\% = \mathrm{e}^{-\pi \zeta / \sqrt{1-\zeta^2}} \times 100 \\% = 32.96 \\% $<br>

  系统实际超调量 $\sigma \\%= 13\\%$

+ 调节时间

  一阶系统调节时间 $t_{s1} = 3 T =1.58 \ s$<br>

  欠阻尼系统调节时间 $t_{s2} = \frac{3.5}{\zeta \omega_n} = 2.24 \ s$<br>

  系统实际调节时间 $t_s = 1.85 \ s$

（2）稳态性能指标

开环系统在坐标原点有一个极点， 所以系统属 I 型系统，因而系统可以无误差跟踪阶跃信号

