### Interval 1: Acceleration phase, jerk limited acceleration
$$
\begin{aligned}
v_0 &\in \mathbb{R}\\
a_0 &\in \mathbb{R}\\
s_0 &= 0\\
\overset{[0]}{t_f} &= 0\\
t &\in (\overset{[0]}{t_f}, \overset{[1]}{t_f}] \subset \mathbb{R} \\
\end{aligned}
$$
$$
\begin{aligned}
\overset{[1]}{j}(t) &= j_{max} \\
\overset{[1]}{a}(t) &= \int_{0}^{\overset{[1]}{t_f}}{\overset{[1]}{j}(t)\,dt} = j_{max} \, t + a_{0} \\
\overset{[1]}{v}(t) &= \int_{0}^{\overset{[1]}{t_f}}{\overset{[1]}{a}(t)\,dt} = \frac{j_{max} \, t^2}{2} + a_0 \, t + v_{0} \\
\overset{[1]}{s}(t) &= \int_{0}^{\overset{[1]}{t_f}}{\overset{[1]}{v}(t)\,dt} = \frac{j_{max} \, t^3}{6} + \frac{a_{0} \, t^2}{2} + v_{0} \, t + s_0
\\
\overset{[1]}{\Delta{}t} &= \overset{[1]}{t_f} - 0 \\
\overset{[1]}{\Delta{}a} &= j_{max} \, \overset{[1]}{\Delta{}t} + a_0 \\
\overset{[1]}{\Delta{}v} &= \frac{j_{max} \, (\overset{[1]}{\Delta{}t})^2}{2} + a_0 \, \overset{[1]}{\Delta{}t} + v_{0} \\
\overset{[1]}{\Delta{}s} &= \frac{j_{max} \, (\overset{[1]}{\Delta{}t})^3}{6} + \frac{a_{0} \, (\overset{[1]}{\Delta{}t})^2}{2} + v_{0} \, \overset{[1]}{\Delta{}t} \\
\end{aligned}
$$

### Interval 2: Acceleration phase, constant acceleration


## Analytical solution of space travel with initial and final accelerations

$$
\begin{aligned}
\overset{[n]}{s}(t) &= \int_{0}^{\overset{[n]}{t_f}}{\overset{[n]}{v}(t)\,dt} = \frac{j_{max} \, t^3}{6} + \frac{\overset{[n]}{a_{0}} \, t^2}{2} + \overset{[n]}{v_{0}} \, t + \overset{[n]}{s_{0}}
\end{aligned}
$$

Then

$$
\begin{aligned}
\overset{[n]}{s_{1}} &= \frac{j_{max} \cdot (\overset{[n]}{t_{f}})^3}{6} + \frac{\overset{[n]}{a_0} \cdot (\overset{[n]}{t_{f}})^2}{2} + \overset{[n]}{v_0} \cdot \overset{[n]}{t_{f}}
\end{aligned}
$$

With:

$$
\begin{aligned}
j_0 &= \frac{j_{max}}{6} \\
a_0 &= \frac{\overset{[n]}{a_0}}{2} \\
t_f &= \overset{[n]}{t_f} \\
\end{aligned}
$$

Then reformulated:
$$
\begin{aligned}
\overset{[n]}{s_1} &= j_0 \, t_f^3 + a_0 \, t_f^2 + v_0 \, t_f \\
\end{aligned}
$$

