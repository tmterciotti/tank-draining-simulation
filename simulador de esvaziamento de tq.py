import numpy as np
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt

# =========================
# CLASSE PID
# =========================
class PID:
    """
    Controlador PID clássico:
    u = Kp*e + Ki∫e dt + Kd de/dt
    """
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral = 0
        self.prev_error = 0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = (self.Kp * error +
                  self.Ki * self.integral +
                  self.Kd * derivative)

        self.prev_error = error
        return output


# =========================
# APP PRINCIPAL
# =========================
class TankApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Tank Simulator (Batch Mode)")

        self.create_widgets()

    # =========================
    # INTERFACE
    # =========================
    def create_widgets(self):
        frame = tk.Frame(self.root)
        frame.pack(padx=10, pady=10)

        def add(label, var):
            tk.Label(frame, text=label).pack()
            tk.Entry(frame, textvariable=var).pack()

        # Entradas
        self.d_tank = tk.DoubleVar(value=2.0)
        self.h0 = tk.DoubleVar(value=15.0)
        self.d_drain_in = tk.DoubleVar(value=2.0)

        self.setpoint = tk.DoubleVar(value=20.0)

        self.Kp = tk.DoubleVar(value=1.0)
        self.Ki = tk.DoubleVar(value=0.5)
        self.Kd = tk.DoubleVar(value=0.1)

        self.valve_manual = tk.DoubleVar(value=0.5)
        self.mode = tk.StringVar(value="PID")

        add("Tank Diameter (m)", self.d_tank)
        add("Initial Height (m)", self.h0)
        add("Drain Diameter (in)", self.d_drain_in)

        add("Flow Setpoint (m³/h)", self.setpoint)

        add("Kp", self.Kp)
        add("Ki", self.Ki)
        add("Kd", self.Kd)

        add("Manual Valve (0–1)", self.valve_manual)

        ttk.Combobox(frame, textvariable=self.mode,
                     values=["PID", "Manual"]).pack(pady=5)

        tk.Button(frame, text="Run Simulation", command=self.run_simulation).pack(pady=10)

        # =========================
        # EQUAÇÕES COM NOMES
        # =========================
        eq = """
Equations Used:

1) Torricelli Equation:
   v = sqrt(2*g*h)

2) Flow Equation:
   Q = Cv * A * sqrt(2*g*h)

3) Mass Balance (Tank):
   dh/dt = -Q / A_tank

4) Volume Calculation:
   V = A_tank * h

5) PID Controller:
   u = Kp*e + Ki∫e dt + Kd de/dt
"""
        tk.Label(frame, text=eq, justify="left").pack(pady=10)

    # =========================
    # SIMULAÇÃO COMPLETA
    # =========================
    def run_simulation(self):

        # ===== PARÂMETROS =====
        g = 9.81
        dt = 0.1

        d_tank = self.d_tank.get()
        h = self.h0.get()
        d_drain = self.d_drain_in.get() * 0.0254

        A_tank = np.pi * (d_tank/2)**2
        A_drain = np.pi * (d_drain/2)**2

        pid = PID(self.Kp.get(), self.Ki.get(), self.Kd.get())

        # ===== HISTÓRICO =====
        tempo = [0]
        altura = [h]
        volume = [A_tank * h]
        vazao = []
        valvula = []

        # =========================
        # LOOP COMPLETO ATÉ ESVAZIAR
        # =========================
        while h > 0:

            # =========================
            # CONTROLE DA VÁLVULA
            # =========================
            if self.mode.get() == "PID":
                flow_actual = vazao[-1] if vazao else 0
                sp = self.setpoint.get() / 3600  # m³/s

                valve = pid.compute(sp, flow_actual, dt)
                valve = max(0, min(1, valve))
            else:
                valve = self.valve_manual.get()

            # =========================
            # 1) TORRICELLI
            # v = sqrt(2gh)
            # =========================
            v = np.sqrt(2 * g * h)

            # =========================
            # 2) FLOW EQUATION
            # Q = valve * A * v
            # =========================
            Q = valve * A_drain * v

            # =========================
            # 3) MASS BALANCE
            # dh/dt = -Q / A_tank
            # =========================
            dhdt = -Q / A_tank

            h = max(h + dhdt * dt, 0)

            # =========================
            # ARMAZENAR
            # =========================
            tempo.append(tempo[-1] + dt)
            altura.append(h)
            volume.append(A_tank * h)
            vazao.append(Q)
            valvula.append(valve)

        # =========================
        # CONVERSÃO
        # =========================
        vazao_m3h = [q * 3600 for q in vazao]

        # =========================
        # PLOT FINAL (CORRIGIDO)
        # =========================
        plt.figure(figsize=(10,6))

        # Volume correto
        plt.plot(tempo, volume, label="Volume (m³)")

        # Vazão
        plt.plot(tempo[:-1], vazao_m3h, label="Flow (m³/h)")

        plt.xlabel("Time (s)")
        plt.title("Tank Draining Simulation")
        plt.legend()
        plt.grid()

        plt.show()


# =========================
# MAIN
# =========================
root = tk.Tk()
app = TankApp(root)
root.mainloop()