#!/usr/bin/env python3
"""
GUI de telemetria para o robô seguidor de linha (ESP32).

Recebe via UART (115200 baud) linhas no formato:
    >DATA:<sens>,<L_mult>,<R_mult>,<L_pid>,<R_pid>,<speed_L>,<speed_R>

    sens    - inteiro com 6 bits dos sensores (bit=1 => linha detectada),
              bit0=esq. extrema, bit1=esq., bit2=meio, bit3=dir., bit4=dir. extrema,
              bit5=flag de parada (sonar).
    L_mult  - multiplicador da roda esquerda (inteiro).
    R_mult  - multiplicador da roda direita  (inteiro).
    L_pid   - saída do PID da roda esquerda (float).
    R_pid   - saída do PID da roda direita  (float).
    speed_L - velocidade estimada da roda esquerda (cm/s, inteiro).
    speed_R - velocidade estimada da roda direita  (cm/s, inteiro).

Mostra:
    - Uma fileira horizontal de "LEDs" representando os bits de eventBits.
    - Os valores L_mult e R_mult abaixo dos LEDs.
    - Dois retângulos (rodas) com os valores L_pid e R_pid dentro.

Dependência: pyserial  ->  pip install pyserial
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

BAUDRATE = 115200
PREFIX = ">DATA:"

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

        self._build_connection_bar()
        self._build_led_row()
        self._build_mult_labels()
        self._build_wheels()

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

    def _build_led_row(self):
        frame = tk.Frame(self.root, bg="#1e1e1e")
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

    def _build_mult_labels(self):
        frame = tk.Frame(self.root, bg="#1e1e1e")
        frame.pack(pady=5)

        self.l_mult_var = tk.StringVar(value="L_mult: --")
        self.r_mult_var = tk.StringVar(value="R_mult: --")

        tk.Label(frame, textvariable=self.l_mult_var, bg="#1e1e1e", fg="white",
                 font=("Consolas", 14, "bold"), width=14).pack(side="left", padx=20)
        tk.Label(frame, textvariable=self.r_mult_var, bg="#1e1e1e", fg="white",
                 font=("Consolas", 14, "bold"), width=14).pack(side="left", padx=20)

    def _build_wheels(self):
        frame = tk.Frame(self.root, bg="#1e1e1e")
        frame.pack(pady=20)

        self.wheel_canvas = tk.Canvas(frame, width=360, height=180,
                                      bg="#1e1e1e", highlightthickness=0)
        self.wheel_canvas.pack()

        # roda esquerda
        self.wheel_canvas.create_text(80, 15, text="Roda Esquerda (L_pid)",
                                      fill="#cccccc", font=("Consolas", 10))
        self.l_wheel = self.wheel_canvas.create_rectangle(30, 30, 130, 160,
                                                          fill="#2c3e50", outline="#3498db",
                                                          width=3)
        self.l_pid_text = self.wheel_canvas.create_text(80, 85, text="--",
                                                        fill="white",
                                                        font=("Consolas", 22, "bold"))
        self.l_speed_text = self.wheel_canvas.create_text(80, 130, text="-- cm/s",
                                                          fill="#f1c40f",
                                                          font=("Consolas", 11))

        # roda direita
        self.wheel_canvas.create_text(280, 15, text="Roda Direita (R_pid)",
                                      fill="#cccccc", font=("Consolas", 10))
        self.r_wheel = self.wheel_canvas.create_rectangle(230, 30, 330, 160,
                                                          fill="#2c3e50", outline="#3498db",
                                                          width=3)
        self.r_pid_text = self.wheel_canvas.create_text(280, 85, text="--",
                                                        fill="white",
                                                        font=("Consolas", 22, "bold"))
        self.r_speed_text = self.wheel_canvas.create_text(280, 130, text="-- cm/s",
                                                          fill="#f1c40f",
                                                          font=("Consolas", 11))

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
            if line.startswith(PREFIX):
                self.data_queue.put(line[len(PREFIX):])

    def _poll_queue(self):
        """Roda na thread da GUI: aplica o último pacote recebido."""
        latest = None
        try:
            while True:
                latest = self.data_queue.get_nowait()
        except queue.Empty:
            pass
        if latest is not None:
            self._update_display(latest)
        self.root.after(50, self._poll_queue)

    # ------------------------------------------------------------- update
    def _update_display(self, payload):
        parts = payload.split(",")
        if len(parts) != 7:
            return
        try:
            sens = int(parts[0])
            l_mult = int(parts[1])
            r_mult = int(parts[2])
            l_pid = float(parts[3])
            r_pid = float(parts[4])
            l_speed = int(parts[5])
            r_speed = int(parts[6])
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

        # multiplicadores
        self.l_mult_var.set(f"L_mult: {l_mult}")
        self.r_mult_var.set(f"R_mult: {r_mult}")

        # rodas (PID + velocidade estimada)
        self.wheel_canvas.itemconfig(self.l_pid_text, text=f"{l_pid:.1f}")
        self.wheel_canvas.itemconfig(self.r_pid_text, text=f"{r_pid:.1f}")
        self.wheel_canvas.itemconfig(self.l_speed_text, text=f"{l_speed} cm/s")
        self.wheel_canvas.itemconfig(self.r_speed_text, text=f"{r_speed} cm/s")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    TelemetryGUI(root)
    root.mainloop()
