# Longitudinal Flight Control of a Fixed-Wing Aircraft
## Overview
+ Aircraft mode and flight quality analysis are primarily based on the full-period dynamic model, with the source code available in `allPeriod.m` <br><br>
+ Flight control, root locus, stability margins, handling quality, and time-domain characteristics analysis are mainly based on the short-period model, with the source code available in `shortPeriod.m` and the Simulink model in `shortPeriodSimulink.slx`
## 1. Modeling

### Parameter Determination

#### 1. Typical Fixed-Wing Civil Aircraft Parameters

+ Aircraft weight: $W = 12224 \ N​$<br>

+ Moments of inertia: $I_x=1420.9 \mathrm{~kg} \cdot \mathrm{~m}^2, I_y=4067.5 \mathrm{~kg} \cdot \mathrm{~m}^2, I_z=4786.0 \mathrm{~kg} \cdot \mathrm{~m}^2, I_{x z}=0 \mathrm{~kg} \cdot \mathrm{~m}^2$<br>

+ Wing area: $S=17.1 \mathrm{~m}^2$<br>

+ Mean aerodynamic chord: $c=1.74 \mathrm{~m}$<br>

+ Wingspan: $b=10.18 \mathrm{~m}$

#### 2. Basic Flight Parameters

+ Aircraft flying at sea level at $M a=0.158$, $C_{L *}=0.41, C_{D *}=0.05$<br>

+ Angle of attack derivatives: $C_{L \alpha}=4.44, C_{D \alpha}=0.33, C_{m \alpha}=-0.683​$<br>

+ Velocity derivatives: $C_{L V}=0.0, C_{D V}=0.0, C_{m V}=0.0$<br>

+ Dynamic derivatives: $C_{L \dot{\alpha}}=C_{z \dot{\alpha}}=0.0, C_{m \dot{\alpha}}=-4.36, C_{L q}=-C_{z q}=3.80, C_{m q}=-9.96$<br>

+ Control derivatives: $C_{L \delta_e}=-C_{z \delta_e}=0.355, C_{D \delta_e}=0.0, C_{m \delta_e}=-0.923$<br>

+ Assuming no variation in thrust with velocity, then $T_V = 0$<br>

+ Gravity acceleration: $g = 9.81 \ m/s^2$<br>

+ Atmospheric density: $\rho = 1.225 \ kg/m^3$

#### 3. Additional Parameters

+ Aircraft mass: $m = \frac{W}{g}$<br>

+ Reference velocity: $V_* = Ma \times 340 $<br>

+ Reference dynamic pressure: $q_* = \frac{\rho {V_*}^2}{2}$<br>

+ Assuming zero wind speed, level straight-line flight with no sideslip $\gamma_* = 0$ ，maintaining constant altitude

### Model Formulation

The modeling is based on the simplified form of the longitudinal perturbation equations, neglecting the influence of altitude variations in perturbation motion and considering only the elevator control for longitudinal flight control. The system is represented in matrix form as:<br>
```math
\frac{\mathrm{d}}{\mathrm{d} t}\left[\begin{array}{c}\Delta V \\ \Delta \alpha \\ \Delta q \\ \Delta \theta\end{array}\right]=\left[\begin{array}{cccc}X_V & X_\alpha+g & 0 & -g \\ -Z_V & -Z_\alpha & 1 & 0 \\ \bar{M}_ V-\bar{M}_ {\dot{\alpha}} Z_V & \bar{M}_ \alpha-\bar{M}_ {\dot{\alpha}} Z_\alpha & \bar{M}_ q+\bar{M}_ {\dot{\alpha}} & 0 \\ 0 & 0 & 1 & 0\end{array}\right]     \left[\begin{array}{c}\Delta V \\ \Delta \alpha \\ \Delta q \\ \Delta \theta\end{array}\right]+\left[\begin{array}{c}X_{\delta_{\mathrm{e}}} \\ -Z_{\delta_{\mathrm{e}}} \\ \bar{M}_ {\delta_{\mathrm{e}}}-\bar{M}_ {\dot{\alpha}} Z_{\delta_{\mathrm{e}}} \\ 0 \end{array}\right]\Delta \delta_{\mathrm{e}}
```
<br>
This formulation involves 11 longitudinal aerodynamic derivatives, given by:<br><br>

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

## 2. Inherent Modal and Flight Quality Analysis

### Inherent Modal Analysis

#### 1. Short-Period Mode (Approximate Model)

+ Natural frequency: $\omega_{n, \ sp }=\sqrt{-\left(\bar{M}_ a+\bar{M}_ q Z_\alpha\right)} = 3.6138$<br>

+ Damping ratio: $\zeta_{\mathrm{sp}}=-\frac{\bar{M}_ q+\bar{M}_ {\dot{\alpha}}-Z_\alpha}{2 \omega_{\mathrm{n} . \mathrm{sp}}} = 0.6954$<br>

+ Period: $T_{sp} = \frac{2\pi}{\omega_{n, \ sp}} = 2.4194$<br>

+ Half decay time: $t_{1 / 2, sp}=-\frac{\ln 2}{\eta_{sp}} = 0.275$<br>

+ Oscillation cycles in half decay period: $N_{1 / 2, sp}=\frac{\ln 2 \sqrt{1-\xi^2_{sp}}}{2 \pi \xi_{sp}} = 0.114$

#### 2. Phugoid Mode

+ Natural frequency: $\omega_{\mathrm{n} , \ \mathrm{p}}= \sqrt{\eta^2_p + \omega_p^2} = 0.2137$<br>

+ Damping ratio: $\zeta_{\mathrm{p}}= \frac{-\eta_p}{w_n, \ p} = 0.0798$<br>

+ Period: $T_p = \frac{2\pi}{\omega_{n, \ p}} = 29.4923$<br>

+ Half decay time: $t_{1 / 2, p}=-\frac{\ln 2}{\eta_p} = 40.6338$<br>

+ Oscillation cycles in half decay period: $N_{1 / 2, p}=\frac{\ln 2 \sqrt{1-\xi^2_{p}}}{2 \pi \xi_{p}} = 1.379$

### Inherent Flight Quality Analysis

The inherent flight quality mainly focuses on the damping ratio of short-period and phugoid modes

#### 1. Short-Period Mode

 $0.35 < \zeta_{\mathrm{sp}} = 0.6954 < 1.3$<br>

The damping ratio for the short-period mode meets the requirements

#### 2. Phugoid Mode

$\zeta_p = 0.0798 \ge 0.04$<br>

The damping ratio for the long-period mode satisfies the requirements 

## 3. $C^*$ Control Law Design

$C^* = n_n+\frac{V_{\mathrm{co}}}{g} q$<br>

Here, $V_{co}$ represents the crossover speed, generally ranging from $120 \sim 132 m/s$，here selected as $V_{co} = 122 m/s$<br>

The control law is designed for the error in $\Delta C^*$, the error is represented as: <br>

$e(t) = \Delta C^{**} - \Delta C^*$<br>

where $\Delta C^{**}$  is the desired value of $\Delta C^*$ <br>

PI control will be applied to the design of the $C^*$ control law, in a form similar to the following:<br>

$u(t) = {K_P e(t) + K_I \int e(t) dt}$<br>

The actuator model is defined as:<br>

$G_\delta(s) = \frac{-1}{0.1s+1}$

## 4. Control System Design and Analysis

### Control System Design

The control of the $C^*$ metric typically needs to be accomplished within a short period, hence the dynamics model is simplified to the short-period model, focusing on elevator control for aircraft:<br>
```math
\left[\begin{array}{c}\Delta \dot{\alpha} \\ \Delta \dot{q}\end{array}\right]=\left[\begin{array}{cc}-Z_\alpha & 1 \\ \bar{M}_\alpha-\bar{M}_{\dot{\alpha}} Z_\alpha & \bar{M}_q+ \bar{M}_{\dot{\alpha}}\end{array}\right]\left[\begin{array}{c}\Delta \alpha \\ \Delta q\end{array}\right] + \left[\begin{array}{c}-Z_{\delta_e} \\ \bar{M}_{\delta_e}-\bar{M}_{\dot{\alpha}} Z_{\delta_e}\end{array}\right] \Delta \delta_e 
```
<br>
The relationship between the change in normal load and the change in angle of attack is:<br><br>

$\Delta n_n = \frac{C_{L\alpha} q S \Delta \alpha}{W}$<br>

The output of the closed-loop system is:<br>

$\Delta C^* = \Delta n_n +\frac{V_{\mathrm{co}}}{g} \Delta q$<br>

The control system consists of a Stability Augmentation System (SAS) and a Control Augmentation System (CAS):

+ S.A.S<br>

  + Normal load feedback with an amplifier gain of $K_{n1}$ <br>

  + Pitch damper with an amplifier gain of $K_{q1}$ and a washout network time constant of $1$<br>

+ C.A.S<br>

  + PI controller with a proportional coefficient $K_P$ and an integral coefficient $K_I$<br>

  + Control amplifier gain $K_a$<br>

  + Command feedforward compensator with an amplifier gain $K_{ff}$, which adjusts a zero in the system without altering the poles<br>

Establish the following control system in Simulink:<br>

![alt](/img/shortPeriodSimulink.jpg)

The output $\Delta C^*$ of the control system is shown below:<br>

![alt](/img/Delta_C_star.jpg)

 $\Delta q$ is depicted in the following graph:

![alt](/img/Delta_q.jpg)

### Root Locus Analysis

Ignoring the feedforward compensator $K_{ff}$ temporarily, consider $K_{n1} = 0.05, K_{q1} = 0.01, K_P = 0.001, K_I = 0.04$<br>

The open-loop transfer function of the system is:<br>

$G_{ol}(s) = K^* \frac{s^3 + 43.817s^2 + 155.497s + 112.68}{s^5 + 16.0256s^4 + 79.6081s^3 + 203.0028s^2 + 137.325s}$<br>

where $K^* = K_G^*  K_H^* = 1.4836 K_a K_H^* = 1.4836 K_a$<br>

Among them,<br>

$K^*$ is the open-loop root locus gain <br>

$K_G^*$ is the forward path root locus gain<br>

$K_H^*$ is the feedback path root locus gain<br>

$K_a$ is the control amplifier gain<br>

Root locus plots are as follows:<br>

![alt](/img/rlocus1.jpg)

![alt](/img/rlocus2.jpg)

![alt](/img/rlocus3.jpg)

From the root locus diagram, it can be seen that when $0 < K^* < 12.5$, the root locus remains in the left half of the $s$-plane, indicating that the system is stable

### Stability Margin Analysis

Nyquist plots are shown below:<br>

![alt](/img/nyquist.jpg)

![alt](/img/nyquist2.jpg)

Bode plots are shown below:<br>

![alt](/img/bode.jpg)

The gain margin is $24.8 dB$, and the phase margin is $87.5°$

### Flight Quality Analysis

(1) $CAP$ Index

With $K_a = 0.35, K_{ff} = 0.03$, the closed-loop transfer function is determined to be:<br>

$G_{cl}(s) = \frac{45.03s^3 + 192.6s^2 + 206.1s + 58.51}{s^5 + 16.03s^4 + 80.14s^3 + 225.8s^2 + 218.2s + 58.54}$<br>

This results in a natural frequency for the short-period mode of $\omega_{n, sp} = 3.67$ and a damping ratio of $\zeta_{n, sp} = 0.61$<br>

The Control Anticipation Parameter ($CAP$) is calculated as:<br>

$CAP = \frac{\omega_{\mathrm{n}, \mathrm{sp}}^2}{(V \star / g) Z_\alpha} = 1.21$<br>

This study primarily focuses on the control during the cruising phase of the aircraft, which is categorized under Type B flight phase. The short-period flight quality requirements for this phase are illustrated in the figure below:<br>

![alt](/img/CAP_CATB.jpg)

It can be observed that the aircraft meets Level 1 flight quality requirements during the cruising phase<br>

(2) $C^*$ Index

When the input is $1g$, the $C^*$ index is depicted in the following figure, demonstrating that the aircraft meets the envelope constraints:<br>

![alt](/img/C_star_index.jpg)

### Time-Domain Characteristics Analysis

（1）Dynamic Performance Indicators<br>

The closed-loop poles are shown in the following figure:<br>

![alt](/img/closed_loop_poles_zeros.jpg)

The closed-loop system has five poles, including three real poles and one pair of complex conjugate poles:<br>

$P_1 (-0.4405, 0)$<br>

$P_2 (-0.9783, 0) $<br>

$P_3 ( -2.254, 2.893)$<br>

$P_4 ( -2.254, -2.893)$<br>

$P_5 (-10.1, 0)$<br>

There are three zeros in the system:<br>

$Z_1 (-2.82, 0)$<br>

$Z_2 (-1, 0)$<br>

$Z_3 (-0.46, 0)$<br>

The pole $P_1$ and zero $Z_3$ , as well as the pole $P_2$ and zero $Z_2$, are located too close to each other near the imaginary axis, which leads to mutual attenuation of their effects on the system response. The distance of pole $P_5​$ from the imaginary axis is relatively greater compared to $P_3, P_4​$. Therefore, the system response is primarily determined by the conjugate pole pair $P_3, P_4​$ and the zero $Z_1​$<br>

The complex conjugate poles $P_3， P_4$ represent an underdamped system with natural frequency $w_n = 3.67$ and damping ratio $\zeta = 0.61$<br>

The zero $Z_1$ mainly reduces peak time, accelerating system response, but increases overshoot<br>

Response to Step Input:<br>

+ Rise Time:

  Rise time of the underdamped system: $t_{r1}=\frac{\pi-arccos\zeta}{\omega_d} = 0.77 \ s$<br>

  Actual rise time: $t_r = 0.31 \ s$<br>

+ Peak Time:

  Peak time of the underdamped system: $t_{p1} = 1.09 \ s$<br>

  Actual peak time: $t_p = 0.74 \ s$<br>

+ Overshoot:

  Overshoot of the underdamped system: $\sigma_1 \\% = \mathrm{e}^{-\pi \zeta / \sqrt{1-\zeta^2}} \times 100 \\% = 8.7 \\%$<br>

  Actual overshoot $\sigma \\%= 18 \\%$<br>

+ Settling Time:

  Settling time of the underdamped system: $t_{s1} = \frac{3.5}{\zeta \omega_n} = 1.55 \ s$<br>

  Actual settling time: $t_s = 2.45 \ s$<br>

（2）Steady-State Performance Indicators<br>

The open-loop system has a pole at the origin, categorizing it as a Type I system, which allows it to track a step input with zero steady-state error

