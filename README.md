# tank-draining-simulation
Simulation of tank draining with PID control using Python

# Tank Draining Simulation with PID

This project simulates the draining of a cylindrical tank using gravity.

## Features
- Dynamic simulation of tank draining
- Flow rate calculation using Torricelli equation
- PID control of outlet valve
- Manual valve mode

## Equations

### Torricelli Equation
v = sqrt(2gh)

### Flow Rate
Q = A * v

### Mass Balance
dh/dt = -Q / A_tank

### Volume
V = A_tank * h

### PID Control
u = Kp*e + Ki∫e dt + Kd de/dt

## Author
Thiago Macedo Terciotti
