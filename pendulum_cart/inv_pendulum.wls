#!/usr/bin/env wolframscript

$Version

fv := (m* g * Cos[x]* Sin[x] + m * L * w * Sin[x] - b*v + u)/(M + m (1 - Cos[x]^2))
dv := D[fv,x]

Print ["dv/dx"]
Print[ToString @ TeXForm[dv]]

fw := (- g *(M + m) * Sin[x] - Cos[x]*(m* L * Sin[x] * w - b * v + u))/(L * (M + m (1 - Cos[x]^2)))
dw := D[fw,x]

Print ["\nd omega/dx"]
Print[ToString @ TeXForm[dw]]

fdv := Function[{x,w}, Evaluate[dv]]
fdw := Function[{x,w}, Evaluate[dw]]

Print ["\nFunction Evaluated at x = 0, omega^2 = 0 (Pendulum down)"]
Print[fdv [0,0]]
Print[fdw [0,0]]

Print ["\nFunction Evaluated at x = Pi, omega^2 = 0 (Pendulum up)"]
Print[fdv [Pi,0]]
Print[fdw [Pi,0]]
