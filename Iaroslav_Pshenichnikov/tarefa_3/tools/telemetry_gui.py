#!/usr/bin/env python3
"""
GUI de telemetria para o robô seguidor de linha (ESP32).

Recebe via UART (115200 baud) linhas no formato:
    >DATA:<sens>,<alvo_L>,<alvo_R>,<lin_cmd>,<ang_cmd>,<speed_L>,<speed_R>,<tgt_lin>,<tgt_ang>

    sens     - 6 bits dos sensores (bit=1 => linha detectada), bit5 = parada (sonar).
    alvo_L/R - velocidade-alvo de cada roda (cm/s), saída da cinemática inversa.
    lin_cmd  - comando do PID linear do corpo (cm/s).
    ang_cmd  - comando do PID angular do corpo (rad/s).
    speed_L/R- velocidade estimada de cada roda (cm/s).
    tgt_lin  - setpoint de velocidade linear do corpo (cm/s).
    tgt_ang  - setpoint de velocidade angular do corpo (rad/s).

E também linhas do wheel_task (valores decodificados, aplicados aos motores):
    >WHL:<L_signed>,<R_signed>
    L_signed/R_signed - duty em ticks de PWM com sinal (negativo = ré).

Mostra (esquerda): LEDs dos sensores, seta da velocidade do corpo e as rodas.
Mostra (direita): gráficos no tempo de target vs comando (linear em cima, angular embaixo).

Dependência: pyserial  ->  pip install pyserial
"""

import math
import queue
import threading
import tkinter as tk
from collections import deque
from tkinter import ttk

import serial
import serial.tools.list_ports

BAUDRATE = 115200
PREFIX = ">DATA:"
PREFIX_WHL = ">WHL:"

# Cinemática (igual ao firmware) e escalas da seta de velocidade do corpo
ARROW_HALF_TRACK = 10.0     # cm, = WHEEL_HALF_TRACK_CM
ARROW_MAX_SPEED = 16.0      # cm/s -> comprimento máximo da seta
ARROW_MAX_OMEGA = 1.6       # rad/s -> inclinação máxima
ARROW_MAX_TILT_DEG = 60.0   # inclinação (graus) na velocidade angular máxima

# Gráficos
PLOT_MAXLEN = 200           # amostras mostradas (~10 s a 20 Hz)
PLOT_W = 380
PLOT_H = 190
COLOR_TGT = "#f1c40f"       # amarelo: setpoint (target)
COLOR_CMD = "#1abc9c"       # ciano: comando do PID

# Rótulos dos 6 bits exibidos como LEDs
LED_LABELS = ["L2", "L1", "M", "R1", "R2", "STOP"]
LED_OFF = "#303030"
LED_ON = "#2ecc40"       # verde para sensores de linha
LED_STOP_ON = "#ff4136"  # vermelho para o bit de parada


class TelemetryGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Telemetria - Seguidor de Linha")
        self.root.configure(bg="#1e1e1e")

        self.ser = None
        self.reader_thread = None
        self.running = False
        self.data_queue = queue.Queue()

        # buffers dos gráficos
        self.buf_tgt_lin = deque(maxlen=PLOT_MAXLEN)
        self.buf_lin_cmd = deque(maxlen=PLOT_MAXLEN)
        self.buf_tgt_ang = deque(maxlen=PLOT_MAXLEN)
        self.buf_ang_cmd = deque(maxlen=PLOT_MAXLEN)

        self._build_connection_bar()

        # corpo: coluna esquerda (widgets) + coluna direita (gráficos)
        body = tk.Frame(self.root, bg="#1e1e1e")
        body.pack(fill="both", expand=True)
        left = tk.Frame(body, bg="#1e1e1e")
        left.pack(side="left", anchor="n")
        right = tk.Frame(body, bg="#1e1e1e")
        right.pack(side="left", anchor="n", padx=10)

        self._build_led_row(left)
        self._build_mult_labels(left)
        self._build_arrow(left)
        self._build_wheels(left)
        self._build_plots(right)

        # processa a fila de dados periodicamente na thread da GUI
        self.root.after(50, self._poll_queue)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ----------------------------------------------------------------- UI
    def _build_connection_bar(self):
        bar = tk.Frame(self.root, bg="#1e1e1e")
        bar.pack(fill="x", padx=10, pady=8)

        tk.Label(bar, text="Porta:", bg="#1e1e1e", fg="white").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var, width=18,
                                       state="readonly")
        self.port_combo.pack(side="left", padx=5)

        tk.Button(bar, text="↻", command=self._refresh_ports, width=3).pack(side="left")

        tk.Label(bar, text=f"  {BAUDRATE} baud", bg="#1e1e1e",
                 fg="#aaaaaa").pack(side="left", padx=5)

        self.connect_btn = tk.Button(bar, text="Conectar", command=self._toggle_connection)
        self.connect_btn.pack(side="left", padx=10)

        self.status_lbl = tk.Label(bar, text="Desconectado", bg="#1e1e1e", fg="#ff4136")
        self.status_lbl.pack(side="left", padx=5)

        self._refresh_ports()

    def _build_led_row(self, parent):
        frame = tk.Frame(parent, bg="#1e1e1e")
        frame.pack(pady=15)

        tk.Label(frame, text="eventBits (sensores)", bg="#1e1e1e",
                 fg="#cccccc").pack()

        self.led_canvas = tk.Canvas(frame, width=6 * 60, height=80,
                                    bg="#1e1e1e", highlightthickness=0)
        self.led_canvas.pack()

        self.leds = []
        r = 18
        for i in range(6):
            cx = 30 + i * 60
            cy = 30
            led = self.led_canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                              fill=LED_OFF, outline="#555555", width=2)
            self.led_canvas.create_text(cx, cy + 35, text=LED_LABELS[i],
                                        fill="#cccccc", font=("Consolas", 10))
            self.leds.append(led)

    def _build_mult_labels(self, parent):
        frame = tk.Frame(parent, bg="#1e1e1e")
        frame.pack(pady=5)

        self.l_mult_var = tk.StringVar(value="alvo_L: --")
        self.r_mult_var = tk.StringVar(value="alvo_R: --")

        tk.Label(frame, textvariable=self.l_mult_var, bg="#1e1e1e", fg="white",
                 font=("Consolas", 14, "bold"), width=14).pack(side="left", padx=20)
        tk.Label(frame, textvariable=self.r_mult_var, bg="#1e1e1e", fg="white",
                 font=("Consolas", 14, "bold"), width=14).pack(side="left", padx=20)

    def _build_arrow(self, parent):
        frame = tk.Frame(parent, bg="#1e1e1e")
        frame.pack(pady=10)

        tk.Label(frame, text="Velocidade do corpo (seta: comprimento=v, ângulo=w)",
                 bg="#1e1e1e", fg="#cccccc").pack()

        self.arrow_canvas = tk.Canvas(frame, width=240, height=200,
                                      bg="#1e1e1e", highlightthickness=0)
        self.arrow_canvas.pack()

        self.arrow_cx, self.arrow_cy = 120, 165   # base (pivô) da seta
        # pivô de referência
        self.arrow_canvas.create_oval(self.arrow_cx - 4, self.arrow_cy - 4,
                                      self.arrow_cx + 4, self.arrow_cy + 4,
                                      outline="#555555")
        self.arrow_line = self.arrow_canvas.create_line(
            self.arrow_cx, self.arrow_cy, self.arrow_cx, self.arrow_cy - 30,
            fill="#1abc9c", width=6, arrow=tk.LAST, arrowshape=(16, 20, 7))
        self.arrow_text = self.arrow_canvas.create_text(
            120, 190, text="v=--  w=--", fill="#cccccc", font=("Consolas", 10))

    def _build_wheels(self, parent):
        frame = tk.Frame(parent, bg="#1e1e1e")
        frame.pack(pady=20)

        self.wheel_canvas = tk.Canvas(frame, width=360, height=180,
                                      bg="#1e1e1e", highlightthickness=0)
        self.wheel_canvas.pack()

        # roda esquerda
        self.wheel_canvas.create_text(80, 15, text="Roda Esquerda (lin_cmd)",
                                      fill="#cccccc", font=("Consolas", 10))
        self.l_wheel = self.wheel_canvas.create_rectangle(30, 30, 130, 160,
                                                          fill="#2c3e50", outline="#3498db",
                                                          width=3)
        self.l_pid_text = self.wheel_canvas.create_text(80, 85, text="--",
                                                        fill="white",
                                                        font=("Consolas", 22, "bold"))
        self.l_speed_text = self.wheel_canvas.create_text(80, 125, text="-- cm/s",
                                                          fill="#f1c40f",
                                                          font=("Consolas", 11))
        self.l_pwm_text = self.wheel_canvas.create_text(80, 148, text="PWM: --",
                                                        fill="#e67e22",
                                                        font=("Consolas", 10))

        # roda direita
        self.wheel_canvas.create_text(280, 15, text="Roda Direita (ang_cmd)",
                                      fill="#cccccc", font=("Consolas", 10))
        self.r_wheel = self.wheel_canvas.create_rectangle(230, 30, 330, 160,
                                                          fill="#2c3e50", outline="#3498db",
                                                          width=3)
        self.r_pid_text = self.wheel_canvas.create_text(280, 85, text="--",
                                                        fill="white",
                                                        font=("Consolas", 22, "bold"))
        self.r_speed_text = self.wheel_canvas.create_text(280, 125, text="-- cm/s",
                                                          fill="#f1c40f",
                                                          font=("Consolas", 11))
        self.r_pwm_text = self.wheel_canvas.create_text(280, 148, text="PWM: --",
                                                        fill="#e67e22",
                                                        font=("Consolas", 10))

    def _build_plots(self, parent):
        tk.Label(parent, text="Linear  —  target (amarelo) vs cmd (ciano)  [cm/s]",
                 bg="#1e1e1e", fg="#cccccc").pack(anchor="w")
        self.plot_lin = tk.Canvas(parent, width=PLOT_W, height=PLOT_H,
                                  bg="#161616", highlightthickness=0)
        self.plot_lin.pack(pady=(0, 14))

        tk.Label(parent, text="Angular  —  target (amarelo) vs cmd (ciano)  [rad/s]",
                 bg="#1e1e1e", fg="#cccccc").pack(anchor="w")
        self.plot_ang = tk.Canvas(parent, width=PLOT_W, height=PLOT_H,
                                  bg="#161616", highlightthickness=0)
        self.plot_ang.pack()

    # ------------------------------------------------------------- serial
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            self.status_lbl.config(text="Selecione uma porta", fg="#ff851b")
            return
        try:
            self.ser = serial.Serial(port, BAUDRATE, timeout=1)
        except serial.SerialException as e:
            self.status_lbl.config(text=f"Erro: {e}", fg="#ff4136")
            return

        self.running = True
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

        self.connect_btn.config(text="Desconectar")
        self.status_lbl.config(text=f"Conectado ({port})", fg="#2ecc40")

    def _disconnect(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.connect_btn.config(text="Conectar")
        self.status_lbl.config(text="Desconectado", fg="#ff4136")

    def _read_loop(self):
        """Roda em thread separada: lê linhas e empurra para a fila."""
        while self.running and self.ser:
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if line.startswith(PREFIX) or line.startswith(PREFIX_WHL):
                self.data_queue.put(line)

    def _poll_queue(self):
        """Roda na thread da GUI: aplica o último pacote de cada tipo."""
        latest_data = None
        latest_whl = None
        try:
            while True:
                line = self.data_queue.get_nowait()
                if line.startswith(PREFIX):
                    latest_data = line[len(PREFIX):]
                elif line.startswith(PREFIX_WHL):
                    latest_whl = line[len(PREFIX_WHL):]
        except queue.Empty:
            pass
        if latest_data is not None:
            self._update_display(latest_data)
        if latest_whl is not None:
            self._update_wheel_pwm(latest_whl)
        self.root.after(50, self._poll_queue)

    # ------------------------------------------------------------- update
    def _update_display(self, payload):
        parts = payload.split(",")
        if len(parts) != 9:
            return
        try:
            sens = int(parts[0])
            alvo_l = int(parts[1])
            alvo_r = int(parts[2])
            lin_cmd = float(parts[3])
            ang_cmd = float(parts[4])
            l_speed = float(parts[5])
            r_speed = float(parts[6])
            tgt_lin = float(parts[7])
            tgt_ang = float(parts[8])
        except ValueError:
            return

        # LEDs
        for i in range(6):
            on = (sens >> i) & 1
            if on:
                color = LED_STOP_ON if i == 5 else LED_ON
            else:
                color = LED_OFF
            self.led_canvas.itemconfig(self.leds[i], fill=color)

        # alvos das rodas
        self.l_mult_var.set(f"alvo_L: {alvo_l}")
        self.r_mult_var.set(f"alvo_R: {alvo_r}")

        # rodas (comando do corpo + velocidade estimada)
        self.wheel_canvas.itemconfig(self.l_pid_text, text=f"{lin_cmd:.1f}")
        self.wheel_canvas.itemconfig(self.r_pid_text, text=f"{ang_cmd:.1f}")
        self.wheel_canvas.itemconfig(self.l_speed_text, text=f"{l_speed:.1f} cm/s")
        self.wheel_canvas.itemconfig(self.r_speed_text, text=f"{r_speed:.1f} cm/s")

        # seta da velocidade do corpo
        self._update_arrow(l_speed, r_speed)

        # gráficos: target vs comando (linear em cima, angular embaixo)
        self.buf_tgt_lin.append(tgt_lin)
        self.buf_lin_cmd.append(lin_cmd)
        self.buf_tgt_ang.append(tgt_ang)
        self.buf_ang_cmd.append(ang_cmd)
        self._draw_plot(self.plot_lin, self.buf_tgt_lin, self.buf_lin_cmd, "cm/s")
        self._draw_plot(self.plot_ang, self.buf_tgt_ang, self.buf_ang_cmd, "rad/s")

    def _draw_plot(self, canvas, buf_target, buf_cmd, unit):
        """Desenha duas séries no tempo (target e cmd) com autoescala em Y."""
        canvas.delete("all")
        w, h = PLOT_W, PLOT_H
        left, right, top, bot = 40, w - 8, 12, h - 14

        vals = list(buf_target) + list(buf_cmd)
        if vals:
            vmin, vmax = min(vals), max(vals)
        else:
            vmin, vmax = -1.0, 1.0
        if vmax - vmin < 1e-3:
            vmin -= 1.0
            vmax += 1.0
        span = vmax - vmin
        vmin -= 0.1 * span
        vmax += 0.1 * span

        def yof(v):
            return bot - (v - vmin) / (vmax - vmin) * (bot - top)

        def xof(i, n):
            return left if n <= 1 else left + i / (n - 1) * (right - left)

        # moldura
        canvas.create_rectangle(left, top, right, bot, outline="#444444")
        # linha do zero
        if vmin < 0 < vmax:
            zy = yof(0.0)
            canvas.create_line(left, zy, right, zy, fill="#3a3a3a", dash=(3, 3))
        # marcas em Y (máx / mín)
        canvas.create_text(left - 4, top, text=f"{vmax:.1f}", fill="#888888",
                           anchor="e", font=("Consolas", 7))
        canvas.create_text(left - 4, bot, text=f"{vmin:.1f}", fill="#888888",
                           anchor="e", font=("Consolas", 7))

        # séries
        for buf, color in ((buf_target, COLOR_TGT), (buf_cmd, COLOR_CMD)):
            if len(buf) >= 2:
                n = len(buf)
                pts = []
                for i, v in enumerate(buf):
                    pts.extend([xof(i, n), yof(v)])
                canvas.create_line(*pts, fill=color, width=2)

        # valores atuais
        if buf_target:
            canvas.create_text(right - 4, top + 2, anchor="ne", fill=COLOR_TGT,
                               font=("Consolas", 8),
                               text=f"target={buf_target[-1]:+.2f} {unit}")
        if buf_cmd:
            canvas.create_text(right - 4, top + 15, anchor="ne", fill=COLOR_CMD,
                               font=("Consolas", 8),
                               text=f"cmd={buf_cmd[-1]:+.2f} {unit}")

    def _update_arrow(self, l_speed, r_speed):
        """Seta da velocidade do corpo (cinemática direta das rodas).
        Comprimento ~ velocidade linear v; inclinação ~ velocidade angular w."""
        v = (l_speed + r_speed) / 2.0                       # cm/s
        omega = (l_speed - r_speed) / (2.0 * ARROW_HALF_TRACK)  # rad/s

        length = 15 + min(abs(v) / ARROW_MAX_SPEED, 1.0) * 120
        tilt = max(-1.0, min(1.0, omega / ARROW_MAX_OMEGA)) * ARROW_MAX_TILT_DEG
        theta = math.radians(tilt)                          # >0 => inclina à direita

        forward = 1.0 if v >= 0 else -1.0
        tip_x = self.arrow_cx + forward * length * math.sin(theta)
        tip_y = self.arrow_cy - forward * length * math.cos(theta)
        self.arrow_canvas.coords(self.arrow_line,
                                 self.arrow_cx, self.arrow_cy, tip_x, tip_y)
        self.arrow_canvas.itemconfig(
            self.arrow_text, text=f"v={v:+.1f} cm/s   w={omega:+.2f} rad/s")

    def _update_wheel_pwm(self, payload):
        """Aplica os valores decodificados pelo wheel_task (duty com sinal)."""
        parts = payload.split(",")
        if len(parts) != 2:
            return
        try:
            l_pwm = int(parts[0])
            r_pwm = int(parts[1])
        except ValueError:
            return
        self.wheel_canvas.itemconfig(self.l_pwm_text, text=f"PWM: {l_pwm:+d}")
        self.wheel_canvas.itemconfig(self.r_pwm_text, text=f"PWM: {r_pwm:+d}")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    TelemetryGUI(root)
    root.mainloop()
