import threading, time, json, csv, os, re, struct
from dataclasses import dataclass, asdict
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import serial
from pymodbus.client.sync import ModbusSerialClient   # 2.5.3

# ----------------- data class -----------------
@dataclass
class Step:
    end_criterion: str
    port_pos: int
    target_mass_g: float
    duration_s: int
    init_flow_ml_min: float
    rev_flow_ml_min: float
    rev_time_s: int
    pre_dwell_after_s: int = 0
    dwell_after_s: int = 0
    mass_correction_g: float = 0.0
    density_g_ml: float = 1.0

    @property
    def step_total_seconds(self):
        return int(self.duration_s + self.rev_time_s + self.pre_dwell_after_s + self.dwell_after_s)

# ----------------- balance class -----------------
class ScaleReader(threading.Thread):
    NUM_RE = re.compile(r"([-+]?\d+(?:[.,]\d+)?)")

    def __init__(self, port="COM8", baudrate=9600, parity='N', bytesize=8, stopbits=1):
        super().__init__(daemon=True)
        self.port, self.baudrate = port, baudrate
        self.parity, self.bytesize, self.stopbits = parity, bytesize, stopbits
        self.ser = None
        self.running = False
        self.connected = False
        self.mass_g = None
        self.error_logged = False

    def stop(self):
        self.running = False
        self.connected = False
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        if self.is_alive():
            self.join(timeout=1.0)

    def run(self):
        self.running = True
        try:
            self.ser = serial.Serial(self.port, self.baudrate, parity=self.parity, 
                                     bytesize=self.bytesize, stopbits=self.stopbits, timeout=0.1)
            self.ser.reset_input_buffer()
            self.connected = True
        except Exception as e:
            print(f"Scale Open Error: {e}"); return
            
        while self.running:
            try:
                if self.ser.in_waiting > 100:
                    self.ser.reset_input_buffer()

                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue

                matches = self.NUM_RE.findall(line)
                if not matches:
                    continue

                new_mass = float(matches[-1].replace(',', ''))

                self.mass_g = new_mass
            except Exception as e:
                if not self.error_logged:
                    print(f"Scale Read Error ({self.port}): {e}")
                    self.error_logged = True
            time.sleep(0.01)

# ----------------- pump class -----------------
class Pump:
    REG_FLOW_READ   = 1003 
    REG_FLOW_WRITE  = 1004 
    REG_START       = 1008 
    REG_DIR         = 1009 
    REG_MODE        = 1019 

    def __init__(self, port="COM4", baudrate=9600, slave_id=1, min_delay_s=0.20):
        self.unit = int(slave_id)
        self.cli = ModbusSerialClient(
            method="rtu", port=port, baudrate=baudrate,
            bytesize=8, parity="E", stopbits=1, timeout=3.0
        )
        if not self.cli.connect():
            raise RuntimeError(f"Pump connect failed on {port}")
        time.sleep(0.2)
        self.abs_flow = 1.0
        self.min_delay = float(min_delay_s)
        try:
            rr = self.cli.write_register(self.REG_MODE, 0, unit=self.unit)
            if rr is None or getattr(rr, 'isError', lambda: False)():
                pass
        except Exception:
            pass

    @staticmethod
    def _regs_to_f32_wordswap(regs):
        if not regs or len(regs) != 2: return None
        hi, lo = regs[1], regs[0]
        raw = (hi << 16) | lo
        return struct.unpack(">f", raw.to_bytes(4, "big"))[0]

    @staticmethod
    def _f32_to_regs_hi_lo(val: float):
        b = struct.pack(">f", float(val))
        return [int.from_bytes(b[0:2], "big"), int.from_bytes(b[2:4], "big")]

    def read_flow(self) -> float | None:
        rr = self.cli.read_holding_registers(self.REG_FLOW_READ, 2, unit=self.unit)
        if rr is None or rr.isError(): raise RuntimeError(f"read_flow failed: {rr}")
        return self._regs_to_f32_wordswap(rr.registers)

    def _write_flow_float(self, value: float):
        regs = self._f32_to_regs_hi_lo(value)
       
        time.sleep(self.min_delay)
        last_err = None
        for attempt in range(1, 6):
            try:
                rr = self.cli.write_registers(self.REG_FLOW_WRITE, regs, unit=self.unit)
                if rr is not None and (not getattr(rr, 'isError', lambda: False)()):
                    time.sleep(max(0.04, self.min_delay))
                    return
                last_err = rr
            except Exception as e:
                last_err = e
            time.sleep(0.15)
        raise RuntimeError(
            f"set_flow failed (Attempted cmd={value:.4f} mL/min, attempts={attempt}): {last_err}"
        )

    def set_flow(self, ml_min: float):
        v = max(0.01, abs(float(ml_min)))
        self.abs_flow = v
        self._write_flow_float(v)

    def _dir(self, forward: bool):
        want = 1 if forward else 0
        rr = self.cli.write_register(
            self.REG_DIR,
            want,
            unit=self.unit
        )

        if rr is None or rr.isError():
            raise RuntimeError(f"_dir failed: {rr}")

        time.sleep(0.20)

    def start(self, forward=True):
        self._dir(forward); time.sleep(0.5)
        self._write_flow_float(max(1e-3, self.abs_flow)); time.sleep(0.1)
        rr = self.cli.write_register(self.REG_START, 1, unit=self.unit)
        if rr is None or rr.isError():
            time.sleep(0.5)
            rr2 = self.cli.write_register(self.REG_START, 1, unit=self.unit)
            if rr2 is None or rr2.isError(): raise RuntimeError(f"start failed: {rr2}")

    def stop(self):
        r1 = self.cli.write_register(self.REG_START, 0, unit=self.unit)
        time.sleep(self.min_delay)
        r2 = self.cli.write_register(self.REG_START, 0, unit=self.unit)
        if (r1 is None or r1.isError()) and (r2 is None or r2.isError()):
            raise RuntimeError(f"stop failed")

    def close(self):
        try: self.cli.close()
        except: pass
        
# ----------------- valve class -----------------
class Valve:
    def __init__(self, port="COM7", baudrate=9600, addr=1):
        self.addr = str(addr)
        self.ser = None
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            self.ser.reset_input_buffer()
            print(f"Valve connected on {port}")
            self.initialize_hardware()
        except Exception as e:
            print(f"Valve connection error: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def interpret_status(self, status_char):
        try:
            val = ord(status_char) if isinstance(status_char, str) and len(status_char) == 1 else status_char
            if not isinstance(val, int):
                return str(status_char)
            if val & 16:
                return f"에러(Code: {val & 0x0F})"
            if val & 32:
                return "완료"
            return "동작 중(Busy)"
        except Exception:
            return f"알 수 없는 상태({status_char})"

    def send(self, cmd_body, pause=0.2):
        if not self.ser:
            return "?"
        full_cmd = f"/{self.addr}{cmd_body}\r".encode("ascii")
        self.ser.write(full_cmd)
        time.sleep(pause)
        if self.ser.in_waiting > 0:
            resp = self.ser.read_all()
            if len(resp) > 2:
                return chr(resp[2])
        return "?"

    def wait_for_ready(self, timeout=20):
        if not self.ser:
            return "?"
        start_time = time.time()
        while time.time() - start_time < timeout:
            self.ser.write(f"/{self.addr}\r".encode("ascii"))
            time.sleep(0.2)
            if self.ser.in_waiting > 0:
                resp = self.ser.read_all()
                if len(resp) > 2:
                    status = chr(resp[2])
                    if ord(status) & 32:
                        return status
            time.sleep(0.3)
        return "?"

    def initialize_hardware(self):
        print("밸브 하드웨어 초기화 중 (Z1R)...")
        self.send("Z1R", pause=0.5)
        time.sleep(1.0)
        final = self.wait_for_ready(timeout=15)
        print(f"초기화 결과: {self.interpret_status(final)} ({final})")

    def goto_port(self, pos: int):
        print(f"밸브 {pos}번 이동 시작 (Cmd: I{int(pos)}R)...")
        self.send(f"I{int(pos)}R", pause=0.2)
        time.sleep(0.5)
        final = self.wait_for_ready(timeout=15)
        ok = final != "?" and bool(ord(final) & 32)
        print(f"결과: {pos}번 포트 이동 {self.interpret_status(final)}")
        return ok

# ----------------- Fuzzy Control -----------------
class FuzzyControl:
    def __init__(self):
        self.adaptive_gain = 1.0
        self.min_gain = 0.5
        self.max_gain = 3.0
        self.last_error = 0.0

        self.base_centers = {
            'NB': -1.0,
            'NS': -0.2,
            'ZO': 0.0,
            'PS': 0.2,
            'PB': 1.0
        }

    def _trap_membership(self, x, a, b, c, d):
        if x <= a or x >= d: return 0.0
        if b <= x <= c: return 1.0
        if a < x < b: return (x - a) / (b - a)
        if c < x < d: return (d - x) / (d - c)
        return 0.0
    
    def _update_gain(self, current_error, target_flow, current_flow_cmd):
        safe_target = target_flow if target_flow > 1e-6 else 1.0
        safe_current = current_flow_cmd if current_flow_cmd > 1e-6 else 1.0

        if self.last_error * current_error < 0:
            self.adaptive_gain *= 0.8
            
        error_diff = abs(current_error - self.last_error)
        if (abs(current_error) > safe_target * 0.3) and (error_diff < safe_current * 0.1):
            self.adaptive_gain *= 1.1
            
        self.adaptive_gain = max(self.min_gain, min(self.max_gain, self.adaptive_gain))
        
        self.last_error = current_error

    def compute(self, e, flow_set, current_flow_cmd):
        self._update_gain(e, flow_set, current_flow_cmd)

        inf = float('inf')
        e_ranges = {
            'NB': (-inf, -inf, -flow_set * 0.3, -flow_set * 0.15),
            'NS': (-flow_set * 0.2, -flow_set * 0.1, -flow_set * 0.05, 0),
            'ZO': (-flow_set * 0.02, 0, 0, flow_set * 0.02),
            'PS': (0, flow_set * 0.05, flow_set * 0.1, flow_set * 0.2),
            'PB': (flow_set * 0.15, flow_set * 0.3, inf, inf),
        }
        
        e_mems = {l: self._trap_membership(e, *r) for l, r in e_ranges.items()}

        BASE_SCALE = 0.05
        numerator = 0.0
        denominator = 0.0
        for label, membership in e_mems.items():
            numerator += self.base_centers[label] * BASE_SCALE * self.adaptive_gain * membership
            denominator += membership
        
        output = numerator / denominator if denominator > 0 else 0.0
        
        return output

# ----------------- GUI App  -----------------
class App:
    def __init__(self, root):
        self.root = root
        root.title("Particle Synthesis Automation (Fuzzy Control)")
        root.geometry("900x700")
        self.running = False
        self.stop_flag = False
        self.log_csv_path = None
        self.log_csv_fp = None

        frm_comm = ttk.LabelFrame(root, text="통신 설정")
        frm_comm.pack(fill="x", padx=10, pady=6)
        self.var_scale1_port = tk.StringVar(value="COM5")
        self.var_scale2_port = tk.StringVar(value="COM6")
        self.var_scale3_port = tk.StringVar(value="COM7")

        self.var_pump_port  = tk.StringVar(value="COM3")
        self.var_valve_port = tk.StringVar(value="COM4")
        self.var_pump_slave = tk.StringVar(value="1")
        self.var_valve_addr = tk.StringVar(value="1")

        ttk.Label(frm_comm, text="저울1 COM").grid(row=0, column=0, padx=4, pady=4, sticky="e")
        ttk.Entry(frm_comm, textvariable=self.var_scale1_port, width=6).grid(row=0, column=1, padx=4, pady=4)
        ttk.Label(frm_comm, text="저울2 COM").grid(row=0, column=2, padx=4, pady=4, sticky="e")
        ttk.Entry(frm_comm, textvariable=self.var_scale2_port, width=6).grid(row=0, column=3, padx=4, pady=4)
        ttk.Label(frm_comm, text="저울3 COM").grid(row=0, column=4, padx=4, pady=4, sticky="e")
        ttk.Entry(frm_comm, textvariable=self.var_scale3_port, width=6).grid(row=0, column=5, padx=4, pady=4)

        ttk.Label(frm_comm, text="펌프 COM").grid(row=1, column=0, padx=4, pady=4, sticky="e")
        ttk.Entry(frm_comm, textvariable=self.var_pump_port, width=6).grid(row=1, column=1, padx=4, pady=4)

        ttk.Label(frm_comm, text="밸브 COM").grid(row=1, column=2, padx=4, pady=4, sticky="e")
        ttk.Entry(frm_comm, textvariable=self.var_valve_port, width=6).grid(row=1, column=3, padx=4, pady=4)

        frm_step = ttk.LabelFrame(root, text="스텝 입력")
        frm_step.pack(fill="x", padx=10, pady=6)

        self.var_end_criterion = tk.StringVar(value="시간")
        self.var_pos = tk.StringVar(value="1")
        self.var_mass = tk.StringVar(value="0")
        self.var_dur_h = tk.StringVar(value="0")
        self.var_dur_m = tk.StringVar(value="0")
        self.var_dur_s = tk.StringVar(value="0")
        self.var_density = tk.StringVar(value="1.0")

        ttk.Label(frm_step, text="기준").grid(row=0, column=0, padx=2, pady=4, sticky="e")
        ttk.Combobox(frm_step, textvariable=self.var_end_criterion, values=["질량", "시간"], width=6, state="readonly").grid(row=0, column=1, padx=2)
        ttk.Label(frm_step, text="밸브").grid(row=0, column=2, padx=2, pady=4, sticky="e")
        ttk.Combobox(frm_step, textvariable=self.var_pos, values=["1","2","3"], width=4).grid(row=0, column=3, padx=2)
        ttk.Label(frm_step, text="줄일 질량(g)").grid(row=0, column=4, padx=2, pady=4, sticky="e")
        ttk.Entry(frm_step, textvariable=self.var_mass, width=8).grid(row=0, column=5, padx=2)
        ttk.Label(frm_step, text="목표 시간(H:M:S)").grid(row=0, column=6, padx=2, pady=4, sticky="e")
        frm_time_small = ttk.Frame(frm_step)
        frm_time_small.grid(row=0, column=7, padx=2, sticky="w")
        ttk.Entry(frm_time_small, textvariable=self.var_dur_h, width=3).pack(side="left")
        ttk.Label(frm_time_small, text=":").pack(side="left", padx=(2,2))
        ttk.Entry(frm_time_small, textvariable=self.var_dur_m, width=3).pack(side="left")
        ttk.Label(frm_time_small, text=":").pack(side="left", padx=(2,2))
        ttk.Entry(frm_time_small, textvariable=self.var_dur_s, width=3).pack(side="left")

        self.var_density = tk.StringVar(value="1.0")
        self.var_mcorr = tk.StringVar(value="0")

        self.var_rev_h = tk.StringVar(value="0")
        self.var_rev_m = tk.StringVar(value="0")
        self.var_rev_s = tk.StringVar(value="0")

        self.var_pre_dwell_h = tk.StringVar(value="0")
        self.var_pre_dwell_m = tk.StringVar(value="0")
        self.var_pre_dwell_s = tk.StringVar(value="0")
        
        self.var_dwell_h = tk.StringVar(value="0")
        self.var_dwell_m = tk.StringVar(value="0")
        self.var_dwell_s = tk.StringVar(value="0")

        ttk.Label(frm_step, text="밀도(g/mL)").grid(row=1, column=0, padx=2, pady=4, sticky="e")
        ttk.Entry(frm_step, textvariable=self.var_density, width=8).grid(row=1, column=1, padx=2)
        ttk.Label(frm_step, text="보정(g)").grid(row=1, column=2, padx=2, pady=4, sticky="e")
        ttk.Entry(frm_step, textvariable=self.var_mcorr, width=8).grid(row=1, column=3, padx=2)
        ttk.Label(frm_step, text="역방향(H:M:S)").grid(row=1, column=4, padx=2, pady=4, sticky="e")
        frm_time_rev = ttk.Frame(frm_step)
        frm_time_rev.grid(row=1, column=5, padx=2, sticky="w")
        ttk.Entry(frm_time_rev, textvariable=self.var_rev_h, width=3).pack(side="left")
        ttk.Label(frm_time_rev, text=":").pack(side="left")
        ttk.Entry(frm_time_rev, textvariable=self.var_rev_m, width=3).pack(side="left")
        ttk.Label(frm_time_rev, text=":").pack(side="left")
        ttk.Entry(frm_time_rev, textvariable=self.var_rev_s, width=3).pack(side="left")
        ttk.Label(frm_step, text="주입 전 대기(H:M:S)").grid(row=1, column=6, padx=2, pady=4, sticky="e")
        frm_time_pre_dwell = ttk.Frame(frm_step)
        frm_time_pre_dwell.grid(row=1, column=7, padx=2, sticky="w")
        ttk.Entry(frm_time_pre_dwell, textvariable=self.var_pre_dwell_h, width=3).pack(side="left")
        ttk.Label(frm_time_pre_dwell, text=":").pack(side="left")
        ttk.Entry(frm_time_pre_dwell, textvariable=self.var_pre_dwell_m, width=3).pack(side="left")
        ttk.Label(frm_time_pre_dwell, text=":").pack(side="left")
        ttk.Entry(frm_time_pre_dwell, textvariable=self.var_pre_dwell_s, width=3).pack(side="left")
        ttk.Label(frm_step, text="주입 후 대기(H:M:S)").grid(row=1, column=8, padx=2, pady=4, sticky="e")
        frm_time_dwell = ttk.Frame(frm_step)
        frm_time_dwell.grid(row=1, column=9, padx=2, sticky="w")
        ttk.Entry(frm_time_dwell, textvariable=self.var_dwell_h, width=3).pack(side="left")
        ttk.Label(frm_time_dwell, text=":").pack(side="left")
        ttk.Entry(frm_time_dwell, textvariable=self.var_dwell_m, width=3).pack(side="left")
        ttk.Label(frm_time_dwell, text=":").pack(side="left")
        ttk.Entry(frm_time_dwell, textvariable=self.var_dwell_s, width=3).pack(side="left")

        frm_step_btn = ttk.Frame(root)
        frm_step_btn.pack(fill="x", padx=10, pady=4)
        ttk.Button(frm_step_btn, text="추가", command=self.add_step).pack(side="left", padx=4)
        ttk.Button(frm_step_btn, text="수정", command=self.edit_selected_step).pack(side="left", padx=4)
        ttk.Button(frm_step_btn, text="선택 삭제", command=self.del_selected).pack(side="left", padx=4)
        ttk.Button(frm_step_btn, text="모두 삭제", command=self.clear_steps).pack(side="left", padx=4)
        ttk.Button(frm_step_btn, text="목록 위로", command=lambda: self.move_step(-1)).pack(side="left", padx=10)
        ttk.Button(frm_step_btn, text="목록 아래로", command=lambda: self.move_step(1)).pack(side="left", padx=4)
        ttk.Button(frm_step_btn, text="JSON 저장", command=self.save_json).pack(side="left", padx=10)
        ttk.Button(frm_step_btn, text="JSON 불러오기", command=self.load_json).pack(side="left", padx=4)

        frm_list = ttk.Frame(root)
        frm_list.pack(fill="x", expand=False, padx=10, pady=6)
        cols = ("#", "기준", "밸브", "질량(g)", "목표 시간", "밀도(g/mL)", "보정(g)", "역방향 시간", "주입 전 대기", "주입 후 대기")
        self.tree = ttk.Treeview(frm_list, columns=cols, show="headings", height=5)
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=40 if c == "#" else 80, anchor="center")
        self.tree.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(frm_list, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.steps = []

        frm_run = ttk.Frame(root)
        frm_run.pack(fill="x", padx=10, pady=6)
        ttk.Button(frm_run, text="실행 시작", width=14, command=self.start_run).pack(side="left", padx=4)
        ttk.Button(frm_run, text="긴급 정지", width=14, command=self.emergency_stop).pack(side="left", padx=4)

        self.recipe_total_var = tk.StringVar(value="예상: 0:00:00")
        ttk.Label(frm_run, textvariable=self.recipe_total_var, font=("Consolas", 12)).pack(side="right", padx=10)
        self.elapsed_var = tk.StringVar(value="경과: 0:00:00")
        ttk.Label(frm_run, textvariable=self.elapsed_var, font=("Consolas", 12)).pack(side="right", padx=10)
        self.total_var = tk.StringVar(value="스텝: 0")
        ttk.Label(frm_run, textvariable=self.total_var, font=("Consolas", 12)).pack(side="right", padx=12)
        self.run_status_var = tk.StringVar(value="대기 중")
        ttk.Label(frm_run, textvariable=self.run_status_var, font=(10), foreground="blue").pack(side="right", padx=20)

        frm_log = ttk.LabelFrame(root, text="로그")
        frm_log.pack(fill="both", expand=True, padx=10, pady=6)
        scrollbar = ttk.Scrollbar(frm_log, orient="vertical")
        self.txt = tk.Text(frm_log, height=12, state="disabled", yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.txt.yview)
        scrollbar.pack(side="right", fill="y")
        self.txt.pack(side="left", fill="both", expand=True)

        self.scale = None
        self.pump = None
        self.valve = None
        self.log_lines = []
        self.run_output_dir = None

    def log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        self.log_lines.append(line) 
        self.txt.configure(state="normal")
        self.txt.insert("end", f"[{ts}] {msg}\n")
        self.txt.see("end")
        self.txt.configure(state="disabled")

    def refresh_tree(self):
        def _sec_to_hms(seconds):
            h, r = divmod(int(seconds), 3600)
            m, s = divmod(r, 60)
            return f"{h}:{m:02d}:{s:02d}"
        for iid in self.tree.get_children(): self.tree.delete(iid)
        for i, st in enumerate(self.steps, start=1):
            dur_str = _sec_to_hms(st.duration_s)
            rev_str = _sec_to_hms(st.rev_time_s)
            pre_dwell_str = _sec_to_hms(st.pre_dwell_after_s)
            dwell_str = _sec_to_hms(st.dwell_after_s)

            self.tree.insert("", "end", values=(
                i, st.end_criterion, st.port_pos, st.target_mass_g,
                dur_str,
                st.density_g_ml,
                st.mass_correction_g,
                rev_str,
                pre_dwell_str,
                dwell_str
            ))

        total_seconds = sum(s.step_total_seconds for s in self.steps)
        h, r = divmod(total_seconds, 3600)
        m, s = divmod(r, 60)
        
        self.recipe_total_var.set(f"예상: {h}:{m:02d}:{s:02d}")
        self.total_var.set(f"스텝: i")

    def _step_from_inputs(self):
        dur = int(self.var_dur_h.get()) * 3600 + int(self.var_dur_m.get()) * 60 + int(self.var_dur_s.get())
        if dur <= 0:
            raise ValueError("시간은 0보다 커야 합니다.")
        rev_s = int(self.var_rev_h.get()) * 3600 + int(self.var_rev_m.get()) * 60 + int(self.var_rev_s.get())
        pre_dwell_s = int(self.var_pre_dwell_h.get()) * 3600 + int(self.var_pre_dwell_m.get()) * 60 + int(self.var_pre_dwell_s.get())
        dwell_s = int(self.var_dwell_h.get()) * 3600 + int(self.var_dwell_m.get()) * 60 + int(self.var_dwell_s.get())
        target_mass = float(self.var_mass.get())
        mass_correction = float(self.var_mcorr.get())
        density = float(self.var_density.get()) or 1.0
        init_flow = ((target_mass + mass_correction) / density) * 60.0 / dur
        if init_flow > 5.0:
            messagebox.showwarning("유량 제한", "초기 유량이 최대치(5.0 mL/min)를 초과하여 최대값으로 자동 설정됩니다.")
            init_flow = 5.0
        elif init_flow < 0.01:
            init_flow = 0.01
        return Step(
            end_criterion=self.var_end_criterion.get(),
            port_pos=int(self.var_pos.get()),
            target_mass_g=target_mass,
            duration_s=dur,
            init_flow_ml_min=init_flow,
            rev_flow_ml_min=5.0,
            rev_time_s=rev_s,
            pre_dwell_after_s=pre_dwell_s,
            dwell_after_s=dwell_s,
            mass_correction_g=mass_correction,
            density_g_ml=float(self.var_density.get())
        )

    def add_step(self):
        try:
            self.steps.append(self._step_from_inputs())
            self.refresh_tree()
        except Exception as e:
            messagebox.showerror("입력 오류", f"{e}")

    def edit_selected_step(self):
        selection = self.tree.selection()
        if not selection:
            messagebox.showwarning("스텝 선택", "수정할 스텝을 먼저 선택해주세요.")
            return

        index = self.tree.index(selection[0])
        try:
            updated_step = self._step_from_inputs()
            self.steps[index] = updated_step
            self.refresh_tree()
            updated_iid = self.tree.get_children()[index]
            self.tree.selection_set(updated_iid)
            self.tree.focus(updated_iid)
        except Exception as e:
            messagebox.showerror("입력 오류", f"{e}")

    def del_selected(self):
        sel = self.tree.selection()
        if not sel: return
        idxs = sorted([self.tree.index(s) for s in sel], reverse=True)
        for i in idxs: del self.steps[i]
        self.refresh_tree()

    def clear_steps(self):
        if messagebox.askyesno("확인", "모든 스텝을 삭제할까요?"):
            self.steps.clear(); self.refresh_tree()

    def move_step(self, delta):
        selection = self.tree.selection()
        if not selection:
            return

        index = self.tree.index(selection[0])
        new_index = index + delta
        if 0 <= new_index < len(self.steps):
            self.steps[index], self.steps[new_index] = self.steps[new_index], self.steps[index]
            self.refresh_tree()
            new_iid = self.tree.get_children()[new_index]
            self.tree.selection_set(new_iid)
            self.tree.focus(new_iid)

    def save_json(self):
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON","*.json")])
        if path:
            with open(path, "w") as f: json.dump([asdict(s) for s in self.steps], f, indent=2)
            self.log(f"레시피 저장됨: {path}")

    def load_json(self):
        path = filedialog.askopenfilename(filetypes=[("JSON","*.json")])
        if path:
            with open(path, "r") as f:
                data = json.load(f)
                self.steps = [Step(**d) for d in data]
            self.refresh_tree()
            self.log(f"레시피 불러옴: {path}")

    def _get_scale_port_map(self):
        return {
            1: self.var_scale1_port.get().strip(),
            2: self.var_scale2_port.get().strip(),
            3: self.var_scale3_port.get().strip(),
        }

    def _gather_used_ports(self):
        return {st.port_pos for st in self.steps}

    def _stop_scales(self):
        scales = getattr(self, 'scales', None)
        if not scales:
            return
        for s in list(scales.values()):
            try:
                s.stop()
            except Exception:
                pass
        self.scales = {}

    def _close_hardware(self):
        try:
            self._stop_scales()
        except Exception:
            pass
        try: self.pump.stop()
        except: pass
        try: self.pump.close()
        except: pass
        try: self.valve.close()
        except: pass

    def _ensure_scale_for_step(self, step_port, port_map):
        if self.scales is None:
            self.scales = {}

        if step_port in self.scales:
            sc = self.scales[step_port]
            if getattr(sc, "connected", False) or (getattr(sc, "ser", None) and getattr(sc.ser, "is_open", False)):
                return []

        self._stop_scales()
        self.scales = {}

        port_name = port_map.get(step_port)
        if not port_name:
            raise RuntimeError(f"저울 {step_port}의 COM 포트가 설정되지 않았습니다.")

        self.log(f"스텝용 저울 {step_port} 연결 시도 ({port_name})...")
        sc = ScaleReader(port=port_name)
        sc.start()
        self.scales[step_port] = sc
        time.sleep(1.0)

        check_start = time.time()
        while time.time() - check_start < 3:
            if sc.connected or (sc.ser and sc.ser.is_open):
                self.log(f"저울 {step_port} serial 포트 연결 확인: {port_name}")
                return []
            time.sleep(0.2)

        return [f"저울{step_port}"]

    def _open_pump(self):
        port_name = self.var_pump_port.get().strip()
        if not port_name:
            return ["펌프"]
        
        if (self.pump
                and getattr(self.pump, 'cli', None)
                and self.pump.cli.is_socket_open()
                and getattr(self.pump.cli, 'port', None) == port_name):
            try:
                self.pump.read_flow()
                return []
            except Exception:
                self.log(f"펌프({port_name}) - 재연결 시도")

        if self.pump:
            try: self.pump.close()
            except Exception: pass
            self.pump = None
            time.sleep(0.3)
        self.log(f"펌프 연결 시도 ({port_name})...")
        try:
            self.pump = Pump(port=port_name, slave_id=int(self.var_pump_slave.get()))
            self.log(f"펌프 연결 확인: {port_name}")
            return []
        except Exception as e:
            self.log(f"펌프 연결 실패: {e}")
            return [f"펌프({port_name})"]

    def _open_valve(self):
        port_name = self.var_valve_port.get().strip()
        if not port_name:
            return ["밸브"]
        
        if (self.valve
                and getattr(self.valve, 'ser', None)
                and self.valve.ser.is_open
                and self.valve.ser.port == port_name):
            try:
                status = self.valve.wait_for_ready(timeout=2)
                if status != "?":
                    return []
                self.log(f"밸브({port_name})  - 재연결 시도")
            except Exception:
                self.log(f"밸브({port_name}) - 재연결 시도")

        if self.valve:
            try: self.valve.close()
            except Exception: pass
            self.valve = None

        self.log(f"밸브 연결 시도 ({port_name})...")
        try:
            self.valve = Valve(port=port_name, addr=self.var_valve_addr.get())
            if not getattr(self.valve, 'ser', None) or not self.valve.ser.is_open:
                raise RuntimeError("시리얼 포트 열기 실패")
            self.log(f"밸브 연결 확인: {port_name}")
            return []
        except Exception as e:
            self.log(f"밸브 연결 실패: {e}")
            return [f"밸브({port_name})"]

    def _ensure_hardware_for_step(self, st, step_idx, port_map):
        while not self.stop_flag:
            bad_devices = []

            try:
                scale_result = self._ensure_scale_for_step(st.port_pos, port_map)
                bad_devices.extend(scale_result)
            except Exception as e:
                bad_devices.append(str(e))

            bad_devices.extend(self._open_pump())
            bad_devices.extend(self._open_valve())

            bad_devices = [d for d in bad_devices if d]
            if not bad_devices:
                self.log(f"스텝 {step_idx} 시작 전 장비 연결 확인 완료")
                return

            bad_list = sorted(set(bad_devices))
            self.log(f"스텝 {step_idx} 시작 전 연결 실패: " + ", ".join(bad_list))

            if self.stop_flag:
                raise RuntimeError("사용자 요청으로 중단됨")

            retry = messagebox.askyesno(
                "장비 재연결 필요",
                f"스텝 {step_idx} 시작 전 다음 장비의 연결이 필요합니다:\n"
                + "\n".join(bad_list)
                + "\n\n연결 후 다시 시도하시겠습니까?"
            )
            if not retry:
                self.log("사용자가 재연결을 취소하였습니다. 스텝을 종료합니다.")
                raise RuntimeError("실행 중단")

            self.log("장비 재연결 확인을 다시 시도합니다.")

# ----------------- Run thread -----------------
    def _run_thread(self):
        self.log("제어 루프 시작 (Fuzzy Control, 10초 주기)")
        log_history = []
        self.log_lines = []
        self.run_output_dir = os.path.join(os.getcwd(), "로그.", datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.run_output_dir, exist_ok=True)
        self.log(f"자동 저장 폴더: {self.run_output_dir}")
        self.scales = {}
        self.pump = None
        self.valve = None

        def _sec_to_hms(seconds):
            h, r = divmod(int(seconds), 3600)
            m, s = divmod(r, 60)
            return f"{h}:{m:02d}:{s:02d}"

        used_ports = self._gather_used_ports()
        if not used_ports:
            self.log("실행할 스텝이 없습니다.")
            return

        port_map = self._get_scale_port_map()

        try:
            if self.stop_flag:
                raise RuntimeError("사용자 요청으로 중단됨")

            start_mass = 0.0
            start_time_all = time.time()
            recipe_total_mass_out = 0.0
            fuzzy = FuzzyControl()

            for i, st in enumerate(self.steps):
                if self.stop_flag:
                    break

                self.total_var.set(f"스텝: {i+1}")

                if st.pre_dwell_after_s > 0 and not self.stop_flag:
                    self.run_status_var.set("장비 연결 확인 중")
                    self._ensure_hardware_for_step(st, i + 1, port_map)
                    self.run_status_var.set("주입 전 대기 중")
                    self.log(f"[스텝 {i+1}] 주입 전 대기 중 ({_sec_to_hms(st.pre_dwell_after_s)})")
                    pre_dwell_end_time = time.time() + st.pre_dwell_after_s
                    while time.time() < pre_dwell_end_time and not self.stop_flag:
                        now = time.time()
                        elapsed = int(now - start_time_all)
                        eh, er = divmod(elapsed, 3600)
                        em, es = divmod(er, 60)
                        self.elapsed_var.set(f"경과: {eh}:{em:02d}:{es:02d}")
                        time.sleep(0.1)

                if self.stop_flag:
                    break

                self.run_status_var.set("장비 연결 확인 중")
                self._ensure_hardware_for_step(st, i + 1, port_map)

                current_scale = self.scales.get(st.port_pos)
                if current_scale is None:
                    raise RuntimeError(f"연결된 저울 {st.port_pos}이 없습니다. 설정을 확인해주세요.")

                self.run_status_var.set("밸브 이동 중")
                try:
                    status_ok = self.valve.goto_port(st.port_pos)
                    if not status_ok:
                        raise RuntimeError(f"밸브 {st.port_pos} 이동 실패")
                except Exception as e:
                    self.log(f"!!! 밸브 전환 실패: {e}")
                    raise e

                time.sleep(2.0)

                step_start_val = current_scale.mass_g
                if step_start_val is None:
                    for _ in range(10):
                        if current_scale.mass_g is not None:
                            step_start_val = current_scale.mass_g
                            break
                        time.sleep(0.2)
                if step_start_val is None:
                    raise RuntimeError(f"저울 {st.port_pos}의 데이터가 수신되지 않았습니다.")

                self.log(f"[스텝 {i+1}] 주입 전 질량 확인 (Scale {st.port_pos}): {step_start_val:.2f}g")

                if i == 0:
                    start_mass = step_start_val
                    self.log(f"시작 질량(Step 1, Scale {st.port_pos}): {start_mass:.2f}g")
                    log_history.append([0, 1, round(start_mass, 2), 0.0, round(st.init_flow_ml_min, 3), 0.0])

                if st.mass_correction_g > 0:
                    goal_mass = None
                    correction_target_mass = step_start_val - st.mass_correction_g
                    correction_done = False
                    self.log(f"--- 스텝 {i+1} 시작 (현재: {step_start_val:.1f}g, 보정 {st.mass_correction_g:.2f}g 먼저 진행) ---")
                else:
                    goal_mass = step_start_val - st.target_mass_g
                    correction_target_mass = None
                    correction_done = True
                    self.log(f"--- 스텝 {i+1} 시작 (현재: {step_start_val:.1f}g -> 목표: {goal_mass:.1f}g) ---")

                control_interval = 10.0
                step_start_time = time.time()

                mass_out_baseline = step_start_val

                log_history.append([
                    int(step_start_time - start_time_all),
                    i + 1,
                    round(step_start_val, 2),
                    0.0,
                    round(st.init_flow_ml_min, 3),
                    0.0
                ])

                t_end = step_start_time + st.duration_s
                use_time_limit = (st.end_criterion != "질량")
                next_control_t = step_start_time + control_interval
                last_pid_mass, last_pid_time = step_start_val, step_start_time

                mass_confirm_count = 0
                MASS_CONFIRM_REQUIRED = 3

                end_row_already_logged = False

                self.run_status_var.set("실행 중")
                current_flow_cmd = st.init_flow_ml_min
                try:
                    self.pump.set_flow(current_flow_cmd)
                    self.pump.start(forward=True)
                except Exception as e_start:
                    self.log(f"!!! 스텝 {i+1} 시작 명령 실패: {e_start}")
                    raise

                while (not self.stop_flag) and ( (use_time_limit and time.time() < t_end) or (not use_time_limit) ):
                    now = time.time()
                    elapsed = int(now - start_time_all)
                    eh, er = divmod(elapsed, 3600)
                    em, es = divmod(er, 60)
                    self.elapsed_var.set(f"경과: {eh}:{em:02d}:{es:02d}")

                    curr_m = current_scale.mass_g if current_scale.mass_g is not None else last_pid_mass

                    is_time_up = now >= t_end

                    is_mass_reached = (goal_mass is not None) and (curr_m <= goal_mass)

                    if st.end_criterion == "질량":
                        if is_mass_reached:
                            mass_confirm_count += 1
                        else:
                            mass_confirm_count = 0
                        if mass_confirm_count >= MASS_CONFIRM_REQUIRED:
                            self.log(f"질량 도달로 종료 (판독 {MASS_CONFIRM_REQUIRED}회 연속 확인)")
                            log_history.append([
                                int(now - start_time_all),
                                i + 1,
                                round(curr_m, 2),
                                round((last_pid_mass - curr_m) / ((now - last_pid_time)/60.0) if (now - last_pid_time) > 0 else 0.0, 2),
                                round(current_flow_cmd, 3),
                                round(mass_out_baseline - curr_m, 2)
                            ])
                            end_row_already_logged = True
                            break

                    if not correction_done:
                        if curr_m <= correction_target_mass:
                            correction_done = True
                            goal_mass = curr_m - st.target_mass_g
                            t_end = now + st.duration_s
                            self.log(f"보정(g) {st.mass_correction_g:.2f}g 완료 (실측 {curr_m:.2f}g). "
                                     f"Fuzzy 목표 재설정: {goal_mass:.2f}g. Fuzzy 제어 시작")
                            last_pid_mass = curr_m
                            last_pid_time = now
                            next_control_t = now
                            mass_out_baseline = curr_m
                        else:
                            time.sleep(0.05)
                            continue

                    if now >= next_control_t:
                        elapsed_min = (now - last_pid_time) / 60.0
                        meas_g_min = (last_pid_mass - curr_m) / elapsed_min if elapsed_min > 0 else 0.0

                        remaining_time_min = max(0.0, (t_end - now) / 60.0)
                        remaining_mass_g = curr_m - goal_mass
                        realtime_target_g_min = remaining_mass_g / remaining_time_min if remaining_time_min > 0.05 else 0.0
                        realtime_target_g_min = max(0.0, realtime_target_g_min)
                        e = realtime_target_g_min - meas_g_min
                        delta_u = fuzzy.compute(e, realtime_target_g_min, current_flow_cmd)

                        current_flow_cmd += delta_u / st.density_g_ml
                        current_flow_cmd = max(0.01, min(5.0, current_flow_cmd))

                        try:
                            self.pump.set_flow(current_flow_cmd)
                            self.log(f"질량:{curr_m:.2f}g, 목표:{realtime_target_g_min:.2f}g/min, 측정:{meas_g_min:.2f}g/min, 명령:{current_flow_cmd:.3f}mL/min")
                        except Exception as e_pump:
                            self.log(f"!!! 펌프 유량 설정 실패: {e_pump}")
                            raise

                        log_history.append([
                            int(now - start_time_all),
                            i + 1,
                            round(curr_m, 2),
                            round(meas_g_min, 2),
                            round(current_flow_cmd, 3),
                            round(mass_out_baseline - curr_m, 2)
                        ])

                        last_pid_mass = curr_m
                        last_pid_time = now
                        next_control_t += control_interval

                    time.sleep(0.05)

                now_exit = time.time()
                curr_m_exit = current_scale.mass_g if current_scale.mass_g is not None else last_pid_mass
                recipe_total_mass_out += (mass_out_baseline - curr_m_exit)
                if not end_row_already_logged:
                    log_history.append([
                        int(now_exit - start_time_all),
                        i + 1,
                        round(curr_m_exit, 2),
                        round((last_pid_mass - curr_m_exit) / ((now_exit - last_pid_time)/60.0) if (now_exit - last_pid_time) > 0 else 0.0, 2),
                        round(current_flow_cmd, 3),
                        round(mass_out_baseline - curr_m_exit, 2)
                    ])

                try:
                    self.pump.stop()
                except Exception as e_stop:
                    self.log(f"!!! 스텝 {i+1} 종료 정지 명령 실패: {e_stop}")
                    raise
                now_end = time.time()
                final_m = current_scale.mass_g if current_scale.mass_g is not None else 0.0
                if st.end_criterion == "질량":
                    mass_reached = end_row_already_logged or (goal_mass is not None and current_scale.mass_g is not None and current_scale.mass_g <= goal_mass)
                    if mass_reached:
                        self.log(f"--- 스텝 {i+1} 종료 (목표 질량 도달, 최종 질량: {final_m:.2f}g, 목표: {goal_mass:.2f}g) ---")
                        # Report time difference relative to planned t_end
                        time_rem = int(t_end - now_end)
                        if time_rem > 0:
                            self.log(f"[부족분] 목표 시간보다 {time_rem}초 일찍 도달함")
                        elif time_rem < 0:
                            self.log(f"[초과분] 목표 시간보다 {-time_rem}초 늦게 도달함")
                    else:
                        self.log(f"--- 스텝 {i+1} 종료 (중단/미도달: 최종 질량: {final_m:.2f}g, 목표: {goal_mass if goal_mass is not None else '미확정'}) ---")
                else:
                    self.log(f"--- 스텝 {i+1} 종료 (목표 시간 도달) ---")
                    if goal_mass is None:
                        self.log("[참고] 보정 단계 중 시간 종료되어 최종 목표(goal_mass)가 확정되지 않았습니다.")
                    else:
                        mass_rem = final_m - goal_mass
                        if mass_rem > 0:
                            self.log(f"[부족분] 목표 질량까지 {mass_rem:.2f}g 부족함")
                        elif mass_rem < 0:
                            self.log(f"[초과분] 목표 질량보다 {-mass_rem:.2f}g 더 줄었음")

                if st.rev_time_s > 0 and not self.stop_flag:
                    self.log(f"역방향 퍼지 시작 ({_sec_to_hms(st.rev_time_s)})")
                    try:
                        fixed_rev_flow = st.rev_flow_ml_min
                        self.pump.set_flow(fixed_rev_flow)
                        self.pump.start(forward=False)
                        rev_end_time = time.time() + st.rev_time_s
                        while time.time() < rev_end_time and not self.stop_flag:
                            now = time.time()
                            elapsed = int(now - start_time_all)
                            eh, er = divmod(elapsed, 3600)
                            em, es = divmod(er, 60)
                            self.elapsed_var.set(f"경과: {eh}:{em:02d}:{es:02d}")
                            time.sleep(0.1)
                        self.pump.stop()
                        self.log(f"역방향 퍼지 완료 (유량: {fixed_rev_flow:.3f} mL/min)")
                    except Exception as e:
                        self.log(f"역방향 실패: {e}")

                if st.dwell_after_s > 0 and not self.stop_flag:
                    self.log(f"주입 후 대기 중 ({_sec_to_hms(st.dwell_after_s)})")
                    dwell_end_time = time.time() + st.dwell_after_s
                    while time.time() < dwell_end_time and not self.stop_flag:
                        now = time.time()
                        elapsed = int(now - start_time_all)
                        eh, er = divmod(elapsed, 3600)
                        em, es = divmod(er, 60)
                        self.elapsed_var.set(f"경과: {eh}:{em:02d}:{es:02d}")
                        time.sleep(0.1)

                self._auto_save_step_outputs(i + 1, log_history)

            self.log("레시피 완료")
            self._auto_save_step_outputs(len(self.steps), log_history)

        except Exception as e:
            self.log(f"중단됨: {e}")
            self._auto_save_step_outputs(len(log_history) and log_history[-1][1] or 0, log_history)
            messagebox.showerror("오류", f"루프 중단: {e}")
        finally:
            self.cleanup()

    def _auto_save_step_outputs(self, step_idx, log_history):
        if not self.run_output_dir:
            return
        try:
            txt_path = os.path.join(self.run_output_dir, "log.txt")
            with open(txt_path, 'w', encoding='utf-8') as f:
                f.write("\n".join(self.log_lines))

            csv_path = os.path.join(self.run_output_dir, "flow_data.csv")
            with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
                writer = csv.writer(f)
                writer.writerow(["time", "step", "질량(g)", "측정 유량(g/min)", "명령 유량(mL/min)", "유출 질량(g)"])
                writer.writerows(log_history)

            self.log(f"스텝 {step_idx} 종료 - 로그/유량 자동 저장 완료")
        except Exception as e:
            self.log(f"자동 저장 실패: {e}")

    def cleanup(self):
        self.run_status_var.set("종료 중...")
        try: self.pump.stop()
        except: pass

        pump_stop_ok = False
        try:
            self.pump.stop()
            pump_stop_ok = True
        except Exception as e:
            self.log(f"!!! 경고: 펌프 정지 실패: {e}")
  
        try:
            self._stop_scales()
        except Exception:
            pass
        
        try: self.pump.close()
        except: pass
        try: self.valve.close()
        except: pass
        self.running = False
        self.run_status_var.set("대기 중")

    def emergency_stop(self):
        self.stop_flag = True
        self.log("!!! 긴급 정지 !!!")

    def start_run(self):
        if not self.steps: return
        if self.running: return
        self.running = True
        self.stop_flag = False
        threading.Thread(target=self._run_thread, daemon=True).start()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
