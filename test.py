import numpy as np
import sympy
import sympy.plotting as plt



t = sympy.symbols("t")
uv = 2
uf = 1
TH_ = uv*sympy.tan(uf)
TH = sympy.integrate(TH_, t)
x_ = uv * sympy.cos(TH)
y_ = uv * sympy.sin(TH)
x = sympy.integrate(x_, t)
y = sympy.integrate(y_, t)

plt.plot_parametric((x, x_), (t, 0, 10))
