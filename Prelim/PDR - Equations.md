

vector originating at B and terminating at C measured with respect to frame A

direction cosine matrix with columns that are the unit-basis vectors of frame B measured with respect to frame A

angular velocity of coordinate frame B relative to frame C measured with respect to frame A

matrix of moments of inertia and products of inertia of body B measured with respect to frame A

vector of first mass moments of body B measured in with respect to frame A

mass of body/particle i

translational momentum of body/particle i measured with respect to frame A

angular momentum of body/particle i about the origin of frame B measured with respect to frame A

force applied to body/particle i measured with respect to frame A

moment on body/particle i about point B measured with respect to frame A

vector of system forces along the system coordinates ${\boldsymbol a}$

***
Equations of motion:
$$
\boldsymbol{H}(\boldsymbol{\gamma})\ddot{\boldsymbol{\gamma}} + \boldsymbol{d}(\boldsymbol{\gamma}, \dot{\boldsymbol{\gamma}}) = \boldsymbol{F}_{\boldsymbol{\gamma}}
$$


***
position and orientation of Atlas:

$$
{^{I}_{I}{\boldsymbol{r}}_{F}} = \begin{bmatrix}\delta x \\ \delta y \\ \delta z\end{bmatrix}
$$
$$
{^{I}\boldsymbol{T}}_{\!F} = \left[\begin{array}{ccc} \cos\left(\psi \right)\,\cos\left(\theta \right) & -\cos\left(\theta \right)\,\sin\left(\psi \right) & \sin\left(\theta \right)\\ \cos\left(\phi \right)\,\sin\left(\psi \right)+\cos\left(\psi \right)\,\sin\left(\phi \right)\,\sin\left(\theta \right) & \cos\left(\phi \right)\,\cos\left(\psi \right)-\sin\left(\phi \right)\,\sin\left(\psi \right)\,\sin\left(\theta \right) & -\cos\left(\theta \right)\,\sin\left(\phi \right)\\ \sin\left(\phi \right)\,\sin\left(\psi \right)-\cos\left(\phi \right)\,\cos\left(\psi \right)\,\sin\left(\theta \right) & \cos\left(\psi \right)\,\sin\left(\phi \right)+\cos\left(\phi \right)\,\sin\left(\psi \right)\,\sin\left(\theta \right) & \cos\left(\phi \right)\,\cos\left(\theta \right) \end{array}\right]
$$
Where:
$\delta x$ = displacement in the x direction
$\delta y$ = displacement in the y direction
$\delta z$ = displacement in the z direction
$\phi$ = angle about the x axis
$\theta$ = angle about the y axis
$\psi$ = angle about the z axis

***






$$
\boldsymbol{{^{I}_{I}{\boldsymbol{r}}_{F}}} = \begin{bmatrix}\delta x \\ \delta y \\ \delta z\end{bmatrix}
$$

$$
{^{I}\boldsymbol{T}}_{\!F} = 
$$

$$
\left[\begin{array}{ccc} \cos\left(\psi \right)\,\cos\left(\theta \right) & -\cos\left(\theta \right)\,\sin\left(\psi \right) & \sin\left(\theta \right)\\ \cos\left(\phi \right)\,\sin\left(\psi \right)+\cos\left(\psi \right)\,\sin\left(\phi \right)\,\sin\left(\theta \right) & \cos\left(\phi \right)\,\cos\left(\psi \right)-\sin\left(\phi \right)\,\sin\left(\psi \right)\,\sin\left(\theta \right) & -\cos\left(\theta \right)\,\sin\left(\phi \right)\\ \sin\left(\phi \right)\,\sin\left(\psi \right)-\cos\left(\phi \right)\,\cos\left(\psi \right)\,\sin\left(\theta \right) & \cos\left(\psi \right)\,\sin\left(\phi \right)+\cos\left(\phi \right)\,\sin\left(\psi \right)\,\sin\left(\theta \right) & \cos\left(\phi \right)\,\cos\left(\theta \right) \end{array}\right]
$$

***

$$
{^{I}_{I}{\boldsymbol{r}}_{A3}} = {^{I}_{I}{\boldsymbol{r}}_{F}} + {^{I}\boldsymbol{T}}_{\!F} {^{F}_{F}{\boldsymbol{r}}_{A1}} + {^{I}\boldsymbol{T}}_{\!F} {^{F}\boldsymbol{T}}_{\!A1} {^{A1}_{A1}{\boldsymbol{r}}_{A2}} + {^{I}\boldsymbol{T}}_{\!F} {^{F}\boldsymbol{T}}_{\!A1} {^{A1}\boldsymbol{T}}_{\!A2} {^{A2}_{A2}{\boldsymbol{r}}_{A3}}
$$
$$
{^{I}\boldsymbol{T}}_{\!A3} = {^{I}\boldsymbol{T}}_{\!F} {^{F}\boldsymbol{T}}_{\!A1} {^{A1}\boldsymbol{T}}_{\!A2} {^{A2}\boldsymbol{T}}_{\!A3}
$$

where:
${^{I}_{I}{\boldsymbol{r}}_{1}}$
${_I^I}\mathbit{r}_1$

${^{A}_{B}{\boldsymbol{r}}_{}}$
${_B^A}r$


Planar two link IK:
$$
\theta_{2}= atan2\left(\pm \sqrt{1 - \left( \frac{z_{d}^{2} + y_{d}^{2} - l_{1}^{2} - l_{2}^{2}}{2l_{1}^{2}l_{2}^{2}}\right)^{2}}, \frac{z_{d}^{2} + y_{d}^{2} - l_{1}^{2} - l_{2}^{2}}{2l_{1}^{2}l_{2}^{2}} \right)
$$


$$
\begin{align*}
K_{1} &= l_{1} + l_{2} \cos(\theta_{2})\\
K_{2} &= l_{2}\sin(\theta_{2})
\end{align*}
$$

$$
\theta_{1} = atan2\left( z_{d}, y_{d} \right) - atan2\left( K_{2}, K_{1} \right)
$$

$$
{^{I}_{I}{\boldsymbol{r}}_{A3}} = {^{I}_{I}{\boldsymbol{r}}_{F}} + {^{F}_{F}{\boldsymbol{r}}_{A1}} + {^{A1}_{A1}{\boldsymbol{r}}_{A2}} + {^{A2}_{A2}{\boldsymbol{r}}_{A3}}
$$

$$
{^{I}_{I}{\boldsymbol{r}}_{A3}} = {^{I}_{I}{\boldsymbol{r}}_{F}} + {^{I}\boldsymbol{T}}_{\!F} {^{F}_{F}{\boldsymbol{r}}_{A1}} + {^{I}\boldsymbol{T}}_{\!F} {^{F}\boldsymbol{T}}_{\!A1} {^{A1}_{A1}{\boldsymbol{r}}_{A2}} + {^{I}\boldsymbol{T}}_{\!F} {^{F}\boldsymbol{T}}_{\!A1} {^{A1}\boldsymbol{T}}_{\!A2} {^{A2}_{A2}{\boldsymbol{r}}_{A3}}
$$

$$
{^{I}_{I}{\boldsymbol{r}}_{A3}} = {^{I}_{I}{\boldsymbol{r}}_{F}} + {^{I}\boldsymbol{T}}_{\!F} {^{F}_{F}{\boldsymbol{r}}_{A1}} + {^{I}\boldsymbol{T}}_{\!A1} {^{A1}_{A1}{\boldsymbol{r}}_{A2}} + {^{I}\boldsymbol{T}}_{\!A2} {^{A2}_{A2}{\boldsymbol{r}}_{A3}}
$$


***
Equations of motion

$$
\boldsymbol{H}(\gamma)\ddot{\gamma} + \boldsymbol{d}(\gamma, \dot{\gamma}) = \boldsymbol{F}_{\gamma}
$$

$$
\ddot{\boldsymbol{\gamma}} = \frac{\boldsymbol{F}_\gamma - \boldsymbol{d}(\gamma, \dot{\gamma})}{\boldsymbol{H}(\gamma)}
$$

$$
\ddot{\boldsymbol{\gamma}} = \frac{F_\gamma -\boldsymbol{d}(\gamma, \dot{\gamma}) + G - B\dot{\gamma} - Csign(\dot{\gamma})}{\boldsymbol{H}(\gamma)}
$$

***
### Runge-Kutta
