import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

# ── Workspace limits (match your Arduino isReachable) ─────────────────
X_MIN, X_MAX = -29.0, 29.0
Y_MIN, Y_MAX = -29.0, 29.0   # positive y unreachable with current mounting
Z_MIN, Z_MAX =   0.0, 44.0

HOME_X, HOME_Y, HOME_Z = -3.0, -6.5, 36.75

class RobotUI:

    def __init__(self, root):
        self.root = root
        self.root.title("3-DOF Robot Arm Controller")
        self.root.resizable(False, False)

        self.serial = None
        self.connected = False
        self.reading = False

        self.build_ui()

    # ══════════════════════════════════════════════════════════════════
    # UI CONSTRUCTION
    # ══════════════════════════════════════════════════════════════════

    def build_ui(self):

        # ── Connection bar ─────────────────────────────────────────────
        conn_frame = tk.LabelFrame(self.root, text="Connection", padx=8, pady=6)
        conn_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=8, sticky="ew")

        tk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=4)
        self.port_var = tk.StringVar()
        self.port_menu = ttk.Combobox(conn_frame, textvariable=self.port_var, width=14)
        self.port_menu.grid(row=0, column=1, padx=4)
        self.refresh_ports()

        tk.Button(conn_frame, text="Refresh", command=self.refresh_ports, width=8
                  ).grid(row=0, column=2, padx=4)

        self.conn_btn = tk.Button(conn_frame, text="Connect", command=self.toggle_connection,
                                  width=10, bg="#000000", fg="black")
        self.conn_btn.grid(row=0, column=3, padx=4)

        self.status_label = tk.Label(conn_frame, text="Disconnected", fg="red", width=14)
        self.status_label.grid(row=0, column=4, padx=6)

        # ── Sliders ────────────────────────────────────────────────────
        slider_frame = tk.LabelFrame(self.root, text="Position Control", padx=10, pady=8)
        slider_frame.grid(row=1, column=0, padx=10, pady=4, sticky="nsew")

        self.sliders = {}
        self.slider_labels = {}

        axes = [
            ("X", X_MIN, X_MAX, HOME_X),
            ("Y", Y_MIN, Y_MAX, HOME_Y),
            ("Z", Z_MIN, Z_MAX, HOME_Z),
        ]

        for i, (axis, mn, mx, home) in enumerate(axes):
            tk.Label(slider_frame, text=f"{axis} (cm)", width=6, anchor="w"
                     ).grid(row=i, column=0, pady=6, padx=4)

            var = tk.DoubleVar(value=home)
            slider = tk.Scale(slider_frame, from_=mn, to=mx,
                              resolution=0.5, orient=tk.HORIZONTAL,
                              variable=var, length=280,
                              command=lambda v, a=axis: self.on_slider(a, v))
            slider.grid(row=i, column=1, padx=6)

            val_label = tk.Label(slider_frame, text=f"{home:.1f}", width=6, anchor="e")
            val_label.grid(row=i, column=2, padx=4)

            self.sliders[axis] = var
            self.slider_labels[axis] = val_label

        # Send slider values button
        tk.Button(slider_frame, text="Send Slider Position",
                  command=self.send_slider_pos,
                  bg="#2196F3", fg="white", width=20
                  ).grid(row=3, column=0, columnspan=3, pady=8)

        # ── Text input ─────────────────────────────────────────────────
        text_frame = tk.LabelFrame(self.root, text="Manual Entry", padx=10, pady=8)
        text_frame.grid(row=2, column=0, padx=10, pady=4, sticky="ew")

        tk.Label(text_frame, text="X:").grid(row=0, column=0, padx=4)
        self.x_entry = tk.Entry(text_frame, width=7)
        self.x_entry.grid(row=0, column=1, padx=4)

        tk.Label(text_frame, text="Y:").grid(row=0, column=2, padx=4)
        self.y_entry = tk.Entry(text_frame, width=7)
        self.y_entry.grid(row=0, column=3, padx=4)

        tk.Label(text_frame, text="Z:").grid(row=0, column=4, padx=4)
        self.z_entry = tk.Entry(text_frame, width=7)
        self.z_entry.grid(row=0, column=5, padx=4)

        tk.Button(text_frame, text="Go", command=self.send_text_pos,
                  bg="#2196F3", fg="white", width=6
                  ).grid(row=0, column=6, padx=8)

        # ── Command buttons ────────────────────────────────────────────
        btn_frame = tk.Frame(self.root, pady=6)
        btn_frame.grid(row=3, column=0, padx=10, sticky="ew")

        tk.Button(btn_frame, text="HOME", command=self.send_home,
                  bg="#FF9800", fg="white", width=12, height=2
                  ).grid(row=0, column=0, padx=6)

        tk.Button(btn_frame, text="Status (P)", command=self.send_status,
                  bg="#9C27B0", fg="white", width=12, height=2
                  ).grid(row=0, column=1, padx=6)

        # ── Serial monitor ─────────────────────────────────────────────
        log_frame = tk.LabelFrame(self.root, text="Serial Monitor", padx=6, pady=6)
        log_frame.grid(row=1, column=1, rowspan=3, padx=10, pady=4, sticky="nsew")

        self.log = tk.Text(log_frame, width=36, height=22, state=tk.DISABLED,
                           bg="#1e1e1e", fg="#d4d4d4", font=("Courier", 9))
        self.log.grid(row=0, column=0)

        scrollbar = tk.Scrollbar(log_frame, command=self.log.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log.config(yscrollcommand=scrollbar.set)

        tk.Button(log_frame, text="Clear", command=self.clear_log
                  ).grid(row=1, column=0, pady=4)

    # ══════════════════════════════════════════════════════════════════
    # CONNECTION
    # ══════════════════════════════════════════════════════════════════

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_menu["values"] = ports
        if ports:
            self.port_menu.current(0)

    def toggle_connection(self):
        if not self.connected:
            try:
                port = self.port_var.get()
                self.serial = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)  # wait for Arduino reset
                self.connected = True
                self.conn_btn.config(text="Disconnect", bg="#f44336")
                self.status_label.config(text="Connected", fg="green")
                self.log_message(f"Connected to {port}\n")
                self.start_reading()
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
        else:
            self.reading = False
            if self.serial:
                self.serial.close()
            self.connected = False
            self.conn_btn.config(text="Connect", bg="#4CAF50")
            self.status_label.config(text="Disconnected", fg="red")
            self.log_message("Disconnected\n")

    # ══════════════════════════════════════════════════════════════════
    # SERIAL READ (background thread)
    # ══════════════════════════════════════════════════════════════════

    def start_reading(self):
        self.reading = True
        thread = threading.Thread(target=self.read_serial, daemon=True)
        thread.start()

    def read_serial(self):
        while self.reading and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode("utf-8", errors="ignore")
                    self.root.after(0, self.log_message, line)
            except:
                break

    # ══════════════════════════════════════════════════════════════════
    # SENDING COMMANDS
    # ══════════════════════════════════════════════════════════════════

    def send_command(self, cmd):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to Arduino first")
            return
        try:
            self.serial.write((cmd + "\n").encode())
            self.log_message(f">> {cmd}\n")
        except Exception as e:
            self.log_message(f"Send error: {e}\n")

    def validate(self, x, y, z):
        if not (X_MIN <= x <= X_MAX):
            messagebox.showwarning("Out of range", f"X must be {X_MIN} to {X_MAX}")
            return False
        if not (Y_MIN <= y <= Y_MAX):
            messagebox.showwarning("Out of range", f"Y must be {Y_MIN} to {Y_MAX}")
            return False
        if not (Z_MIN <= z <= Z_MAX):
            messagebox.showwarning("Out of range", f"Z must be {Z_MIN} to {Z_MAX}")
            return False
        return True

    def send_slider_pos(self):
        x = round(self.sliders["X"].get(), 1)
        y = round(self.sliders["Y"].get(), 1)
        z = round(self.sliders["Z"].get(), 1)
        if self.validate(x, y, z):
            self.send_command(f"{x} {y} {z}")

    def send_text_pos(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Enter numbers for X, Y, Z")
            return
        if self.validate(x, y, z):
            self.send_command(f"{x} {y} {z}")

    def send_home(self):
        self.send_command("H")
        # reset sliders to home position
        self.sliders["X"].set(HOME_X)
        self.sliders["Y"].set(HOME_Y)
        self.sliders["Z"].set(HOME_Z)

    def send_status(self):
        self.send_command("P")

    # ══════════════════════════════════════════════════════════════════
    # SLIDER CALLBACK
    # ══════════════════════════════════════════════════════════════════

    def on_slider(self, axis, value):
        self.slider_labels[axis].config(text=f"{float(value):.1f}")

    # ══════════════════════════════════════════════════════════════════
    # LOGGING
    # ══════════════════════════════════════════════════════════════════

    def log_message(self, msg):
        self.log.config(state=tk.NORMAL)
        self.log.insert(tk.END, msg)
        self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)

    def clear_log(self):
        self.log.config(state=tk.NORMAL)
        self.log.delete(1.0, tk.END)
        self.log.config(state=tk.DISABLED)


# ══════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotUI(root)
    root.mainloop()