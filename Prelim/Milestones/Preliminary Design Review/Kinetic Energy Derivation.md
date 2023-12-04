
total Kinetic energy:
$$
K = \sum\limits_{i = 1}^{N} K_i
$$

Kinetic Energy of each body:
$$
K_{i} = \frac{1}{2}m_{i}V_{i}^{2}
$$
matrix form:
$$
K_{i}= \frac{1}{2} m_{i} \dot{{^{I}_{I}{\boldsymbol{r}}_{i}}^{\intercal}} \dot{{^{I}_{I}{\boldsymbol{r}}_{i}}}
$$
this scan be expanded and simplified:

$$
K_{i} = \underbrace{\frac{1}{2}m_{i} \dot{{^{I}_{I}{\boldsymbol{r}}_{i}}^{\intercal}} \ \dot{{^{I}_{I}{\boldsymbol{r}}_{i}}} }_{\text{translational}} + \underbrace{\frac{1}{2}\dot{{^{I}_{I}{\boldsymbol{r}}_{i}}^{\intercal}} \dot{{^{I}\boldsymbol{T}}_{\!i}^{\intercal}}{^{i}_{i}\boldsymbol{\Gamma}}}_{Coupling} + \underbrace{\frac{1}{2} trace\left(\dot{{^{I}\boldsymbol{T}}_{\!i}} {^{i}_{i}\boldsymbol{J}}\dot{{^{I}\boldsymbol{T}}_{\!i}^{\intercal}}\right)}_{Rotational}
$$

system mass mtx
$$
H(\boldsymbol{\gamma}) = \frac{\partial}{\partial \ \dot{\boldsymbol{\gamma}}} \left( \frac{\partial K}{\partial \ \dot{\boldsymbol{\gamma}}^{\intercal}} \right)
$$



d = jacobian(jacobian(K, gamma_dot).', gamma)*gamma_dot - jacobian(K, gamma).' ;
$$
\boldsymbol{d}(\boldsymbol{\gamma}, \dot{\boldsymbol{\gamma}}) = \left( \frac{\partial}{\partial \ \boldsymbol{\gamma}}  \left( \frac{\partial \ K}{\partial \ \dot{\boldsymbol{\gamma}}}\right)^{\intercal}\right) \dot{\boldsymbol{\gamma}} \ - \left(\frac{\partial \ K}{\partial \ \boldsymbol{\gamma}}\right)^{\intercal}
$$

potential energy
$$
U_{i} = \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix}^{\intercal} {^{I}_{I}{\boldsymbol{r}}_{i}} m_{i} + {^{I}\boldsymbol{T}}_{\!i} \ {^{i}_{i}\boldsymbol{\Gamma}}
$$

total grav potential
$$
\boldsymbol{U} = \sum\limits_{i = 1}^{N} U_{i}
$$

forces due to grav
G = jacobian(U, gamma).';
$$
\boldsymbol{G} = \frac{\partial \ \boldsymbol{U}}{\partial \ \boldsymbol{\gamma}}
$$


firstly, the ground contact force is calculated in the Inertial frame
```
[diag([0, 0, kP])]*r_IIrP + diag([0, 0, kD])*dot_r_IIrP;
```
$$
{^{I}_{} \boldsymbol{\mathrm{F}}_{P}} = \begin{bmatrix}0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & k_{p}\end{bmatrix} {^{I}_{I}{\boldsymbol{r}}_{P}} + \begin{bmatrix}0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & k_{D}\end{bmatrix} \dot{{^{I}_{I}{\boldsymbol{r}}_{P}}}
$$