



$$
Jac_{A3}
$$


$$
\boldsymbol{F} = -1\left(Jac_{A3}^{\intercal} {^{A3}_{A3}{\boldsymbol{r}}_{P}} \times \left({^{I}\boldsymbol{T}}_{\!3}^{\intercal} {^{I}_{} \boldsymbol{\mathrm{F}}_{P}^{\intercal}}\right){^{I}_{} \boldsymbol{\mathrm{F}}_{P}}\right)
$$
TF = -1 *(Jac_3' * [cross(r_33rP, T_IT3' * F_IFP);...

F_IFP]);

F_IFP = [diag([0, 0, kP])]*r_IIrP + diag([0, 0, kD])*dot_r_IIrP;

$$
{^{I}_{} \boldsymbol{\mathrm{F}}_{P}} = \begin{bmatrix}0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & k_{p}\end{bmatrix} {^{I}_{I}{\boldsymbol{r}}_{P}} + \begin{bmatrix}0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & k_{D}\end{bmatrix} \dot{{^{I}_{I}{\boldsymbol{r}}_{P}}}
$$



$$
\boldsymbol{F} = -1\left(Jac_{A3}^{\intercal} \bigg({^{A3}_{A3}{\boldsymbol{r}}_{P}} \times \left({^{I}\boldsymbol{T}}_{\!3}^{\intercal} {^{I}_{} \boldsymbol{\mathrm{F}}_{P}^{\intercal}}\right)\bigg){^{I}_{} \boldsymbol{\mathrm{F}}_{P}}\right)
$$