"""
uwb_recorder.py — UWB Radar CIR data recorder with timer

Usage:
    python uwb_recorder.py                            COM4, 115200, label=empty
    python uwb_recorder.py -t 120                      auto-stop after 120 s
    python uwb_recorder.py -l book -t 300              book label, 5 min
    python uwb_recorder.py COM5 921600 -l empty -t 60  custom port/baud

Commands (while recording):
    label <name>     change scene label (new file)
    status / s       show stats
    stop / q         manual stop
"""

import sys, os, csv, threading, time, datetime

try:
    import serial
except ImportError:
    print("[!] pip install pyserial")
    sys.exit(1)

# ---- platform-specific non-blocking input ----
if sys.platform == "win32":
    import msvcrt
    def kbhit():    return msvcrt.kbhit()
    def getch():    return msvcrt.getwch()
else:
    import select
    def kbhit():
        return select.select([sys.stdin], [], [], 0)[0] != []
    def getch():
        return sys.stdin.read(1)

def readline_nb():
    """Non-blocking line read; returns '' if no full line available."""
    if not kbhit():
        return ""
    line = ""
    while kbhit():
        ch = getch()
        if ch in ("\r", "\n"):
            return line
        if ch in ("\x08", "\x7f"):   # backspace
            line = line[:-1]
            sys.stdout.write("\b \b")
        else:
            line += ch
            sys.stdout.write(ch)
        sys.stdout.flush()
    return line

# =========================================================================
DEFAULT_PORT   = "COM4"
DEFAULT_BAUD   = 115200
DEFAULT_LABEL  = "empty"
SAVE_DIR       = os.path.dirname(os.path.abspath(__file__))
TIMEOUT_READ   = 2.0


class CIRRecorder:
    def __init__(self, port, baud, label, duration_s):
        self.port        = port
        self.baud        = baud
        self.label       = label
        self.duration_s  = duration_s
        self.ser         = None
        self.csv_file    = None
        self.csv_writer  = None
        self.frame_count = 0
        self.running     = True
        self.headers     = None
        self.lock        = threading.Lock()
        self.start_ts    = None
        self.prompt      = ""

    def _new_file(self, first_row=None):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            print(f"\r[FILE] closed ({self.frame_count} frames)")
            self.prompt = ""

        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = os.path.join(SAVE_DIR, f"cir_{self.label}_{ts}.csv")
        self.csv_file = open(fname, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.frame_count = 0
        if self.headers:
            self.csv_writer.writerow(self.headers)
        elif first_row:
            n = len(first_row)
            n_data_cols = n - 7   # minus baseline(6) + label(1)
            if n_data_cols < 0: n_data_cols = 0
            hdr = ["timestamp_ms","seq","fp_index","rx_power","fp_power","noise"]
            if n_data_cols >= 200:   # I/Q format: 128*2=256 columns
                nb = n_data_cols // 2
                for i in range(nb):
                    hdr += [f"i{i}", f"q{i}"]
            else:                    # old magnitude format: ~128 columns
                for i in range(n_data_cols):
                    hdr += [f"cir{i}"]
            hdr += ["label"]
            self.headers = hdr
            self.csv_writer.writerow(hdr)
            print(f"\r[HDR] auto ({nb} CIR bins)")
            self.prompt = ""
        print(f"[FILE] -> {os.path.basename(fname)}")
        return self.csv_file

    def _parse_line(self, raw):
        line = raw.strip("\r\n ")
        if not line: return None
        if line.startswith("["):
            print(f"\r  [MCU] {line}")
            self.prompt = ""
            return None
        if line.startswith("==="):
            print(f"\r  {line}")
            self.prompt = ""
            return None
        if line.startswith("timestamp_ms"):
            self.headers = [h.strip() for h in line.split(",")]
            print(f"\r[HDR] {len(self.headers)} cols  <- {self.headers[0]} ... {self.headers[-1]}")
            self.prompt = ""
            return None
        if line[0].isdigit() or line[0] == "-":
            return [f.strip() for f in line.split(",")]
        return None

    def _reader_thread(self):
        buf = ""
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1).decode(
                    "utf-8", errors="replace")
            except (serial.SerialException, OSError, UnicodeDecodeError):
                time.sleep(0.1); continue
            if not chunk:
                time.sleep(0.01); continue
            buf += chunk
            while "\n" in buf:
                idx = buf.index("\n")
                line = buf[:idx+1]; buf = buf[idx+1:]
                fields = self._parse_line(line)
                if fields is None: continue
                with self.lock:
                    if self.csv_writer is None:
                        self._new_file(first_row=fields)
                    if self.csv_writer:
                        self.csv_writer.writerow(fields)
                        self.frame_count += 1

    def start(self):
        print(f"\n{'='*60}")
        print(f"  UWB Radar CIR Recorder")
        print(f"  Port: {self.port}  |  {self.baud} baud")
        print(f"  Label: {self.label}  |  "
              f"{f'Auto-stop: {self.duration_s}s' if self.duration_s else 'Manual stop'}")
        print(f"  Save: {SAVE_DIR}")
        print(f"{'='*60}")
        print(f"  label <name> | status | stop\n")

        self.ser = serial.Serial(self.port, self.baud, timeout=TIMEOUT_READ)
        self.running = True
        self.start_ts = time.time()
        self.thread = threading.Thread(target=self._reader_thread, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            print(f"\n[FILE] closed ({self.frame_count} frames)")
        elapsed = time.time() - self.start_ts if self.start_ts else 0
        print(f"[STOP] {elapsed:.0f}s  {self.frame_count} frames  "
              f"({self.frame_count/max(1,elapsed):.1f} Hz)\n")

    def change_label(self, new):
        with self.lock:
            old = self.label
            self.label = new.strip()
            if self.csv_writer: self.csv_writer = None
            print(f"\r[LABEL] '{old}' -> '{self.label}'")
            self.prompt = ""

    def status(self):
        elapsed = time.time() - self.start_ts if self.start_ts else 0
        remain = max(0, self.duration_s - elapsed) if self.duration_s else -1
        with self.lock:
            print(f"\r[STATUS] label='{self.label}'  frames={self.frame_count}  "
                  f"elapsed={elapsed:.0f}s  "
                  f"{'T-'+str(int(remain))+'s' if remain>0 else '' if remain==0 else ''}  "
                  f"rate={self.frame_count/max(1,elapsed):.1f} Hz")
            self.prompt = ""

    def elapsed(self):
        return time.time() - self.start_ts if self.start_ts else 0


# =========================================================================
def main():
    port     = DEFAULT_PORT
    baud     = DEFAULT_BAUD
    label    = DEFAULT_LABEL
    duration = 0

    args = sys.argv[1:]
    while args:
        a = args.pop(0)
        if a in ("--label", "-l") and args:
            label = args.pop(0)
        elif a in ("--time", "-t") and args:
            duration = int(args.pop(0))
        elif a.startswith("COM") or a.startswith("/dev/"):
            port = a
        elif a.isdigit():
            baud = int(a)
        else:
            print(f"[!] unknown arg: {a}")
            print("Usage: python uwb_recorder.py [COMx] [baud] [-l LABEL] [-t SECONDS]")
            return

    rec = CIRRecorder(port, baud, label, duration)
    rec.start()

    inbuf = ""
    try:
        while rec.running:
            # ---- timer ----
            if duration > 0 and rec.elapsed() >= duration:
                print(f"\n[TIMER] {duration}s reached, auto-stop.")
                break

            # ---- non-blocking input ----
            line = readline_nb()
            if line:
                cmd = line.strip()
                if not cmd: continue
                parts = cmd.split(None, 1)
                op = parts[0].lower()
                if op in ("stop", "quit", "exit", "q"):
                    break
                elif op == "label" and len(parts) > 1:
                    rec.change_label(parts[1])
                elif op in ("status", "st", "s"):
                    rec.status()
                else:
                    print(f"\r[?] unknown: {cmd}")
                    rec.prompt = ""

            time.sleep(0.1)
    except (KeyboardInterrupt, EOFError):
        print("\n[INT]")
    finally:
        rec.stop()


if __name__ == "__main__":
    main()
