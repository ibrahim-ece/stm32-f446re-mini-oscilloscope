import sys
import time
import threading
import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

PORT = "COM3"
BAUD = 921600
N = 512
FS = 100000                      # 100 kS/s
NFFT = 4096                      # zero-padded FFT length
TIME_DIVS = [0.05, 0.1, 0.2, 0.3]    # ms per division
VOLT_DIVS = [0.1, 0.2, 0.5, 1.0]     # volts per division
SPANS = [5, 10, 20, 50]              # FFT span in kHz

pg.setConfigOptions(antialias=True, background="k", foreground=(200, 200, 200))


class Reader(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, BAUD, timeout=0.5)
        self.latest = None
        self.count = 0
        self.running = True

    def read_frame(self):
        prev = 0
        while self.running:
            b = self.ser.read(1)
            if not b:
                return None
            if prev == 0xAA and b[0] == 0x55:
                raw = self.ser.read(N * 2)
                if len(raw) < N * 2:
                    return None
                return np.frombuffer(raw, dtype="<u2") * 3.3 / 4095
            prev = b[0]
        return None

    def run(self):
        while self.running:
            v = self.read_frame()
            if v is not None:
                self.latest = v
                self.count += 1
        self.ser.close()


def find_trigger(v, level, rising, search=200):
    for i in range(1, search):
        if rising and v[i - 1] < level <= v[i]:
            return i
        if (not rising) and v[i - 1] > level >= v[i]:
            return i
    return 0


def measure(v):
    vmax = float(v.max())
    vmin = float(v.min())
    vpp = vmax - vmin
    out = {"vmax": vmax, "vmin": vmin, "vpp": vpp,
           "freq": None, "period": None, "duty": None}
    rising, falling = [], []

    if vpp > 0.2:
        mid = (vmax + vmin) / 2
        hi, lo = mid + vpp / 4, mid - vpp / 4
        state = v[0] >= hi
        for i in range(1, len(v)):
            if (not state) and v[i] > hi:
                state = True
                j = i
                while j > 0 and v[j - 1] > mid:
                    j -= 1
                if j > 0:
                    rising.append((j - 1) + (mid - v[j - 1]) / (v[j] - v[j - 1]))
            elif state and v[i] < lo:
                state = False
                j = i
                while j > 0 and v[j - 1] < mid:
                    j -= 1
                if j > 0:
                    falling.append((j - 1) + (v[j - 1] - mid) / (v[j - 1] - v[j]))

    seg = v
    if len(rising) >= 2:
        period = (rising[-1] - rising[0]) / (len(rising) - 1) / FS
        out["period"] = period
        out["freq"] = 1.0 / period
        duties = []
        for k in range(len(rising) - 1):
            f = [x for x in falling if rising[k] < x < rising[k + 1]]
            if f:
                duties.append((f[0] - rising[k]) / (rising[k + 1] - rising[k]))
        if duties:
            out["duty"] = 100.0 * sum(duties) / len(duties)
        a, b = int(np.ceil(rising[0])), int(np.floor(rising[-1]))
        if b - a > 2:
            seg = v[a:b]

    out["vavg"] = float(seg.mean())
    out["vrms"] = float(np.sqrt(np.mean(seg ** 2)))
    out["vac"] = float(np.sqrt(np.mean((seg - seg.mean()) ** 2)))
    return out


def spectrum(v):
    """Amplitude spectrum in volts (peak), Hann window, zero-padded."""
    x = v - v.mean()
    w = np.hanning(len(x))
    X = np.fft.rfft(x * w, NFFT)
    amp = np.abs(X) / (w.sum() / 2.0)
    f = np.fft.rfftfreq(NFFT, 1.0 / FS)
    return f, amp


def fmt_freq(f):
    if f is None:
        return "---"
    return f"{f / 1000:.3f} kHz" if f >= 1000 else f"{f:.1f} Hz"


def fmt_period(p):
    if p is None:
        return "---"
    return f"{p * 1000:.3f} ms" if p >= 1e-3 else f"{p * 1e6:.1f} us"


class Scope(QtWidgets.QMainWindow):
    def __init__(self, reader):
        super().__init__()
        self.reader = reader
        self.run = True
        self.level = 1.65
        self.rising = True
        self.time_div = 0.3
        self.volt_div = 0.5
        self.center = 2.0
        self.last_v = None
        self.disp = None
        self.cursors_on = False
        self.fft_on = True
        self.span_khz = 10
        self.W = 300

        self.scroll_on = True
        self.phase = 0.0
        self.scroll_speed = 0.6     # samples per 20 ms tick (raise = faster)
        self.period_samples = None

        self.t = np.arange(self.W) / FS * 1000

        self.setWindowTitle("Mini Oscilloscope - STM32 Nucleo-F446RE")
        self.resize(1300, 840)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # ---- control bar ----
        bar = QtWidgets.QHBoxLayout()
        layout.addLayout(bar)

        self.btn_run = QtWidgets.QPushButton("Stop")
        self.btn_run.clicked.connect(self.toggle_run)
        bar.addWidget(self.btn_run)

        bar.addWidget(QtWidgets.QLabel("Trigger:"))
        self.cb_edge = QtWidgets.QComboBox()
        self.cb_edge.addItems(["Rising", "Falling"])
        self.cb_edge.currentIndexChanged.connect(self.on_edge)
        bar.addWidget(self.cb_edge)

        self.sp_level = QtWidgets.QDoubleSpinBox()
        self.sp_level.setRange(0.0, 3.3)
        self.sp_level.setSingleStep(0.05)
        self.sp_level.setDecimals(2)
        self.sp_level.setSuffix(" V")
        self.sp_level.setValue(self.level)
        self.sp_level.valueChanged.connect(self.set_level)
        bar.addWidget(self.sp_level)

        bar.addWidget(QtWidgets.QLabel("Time/div:"))
        self.cb_time = QtWidgets.QComboBox()
        self.cb_time.addItems([f"{t} ms" for t in TIME_DIVS])
        self.cb_time.setCurrentIndex(3)
        self.cb_time.currentIndexChanged.connect(self.on_scale)
        bar.addWidget(self.cb_time)

        bar.addWidget(QtWidgets.QLabel("Volts/div:"))
        self.cb_volt = QtWidgets.QComboBox()
        self.cb_volt.addItems([f"{v} V" for v in VOLT_DIVS])
        self.cb_volt.setCurrentIndex(2)
        self.cb_volt.currentIndexChanged.connect(self.on_scale)
        bar.addWidget(self.cb_volt)

        bar.addWidget(QtWidgets.QLabel("Centre:"))
        self.sp_center = QtWidgets.QDoubleSpinBox()
        self.sp_center.setRange(0.0, 4.0)
        self.sp_center.setSingleStep(0.1)
        self.sp_center.setDecimals(1)
        self.sp_center.setSuffix(" V")
        self.sp_center.setValue(self.center)
        self.sp_center.valueChanged.connect(self.on_scale)
        bar.addWidget(self.sp_center)

        self.chk_cur = QtWidgets.QCheckBox("Cursors")
        self.chk_cur.toggled.connect(self.show_cursors)
        bar.addWidget(self.chk_cur)

        self.chk_fft = QtWidgets.QCheckBox("FFT")
        self.chk_fft.setChecked(True)
        self.chk_fft.toggled.connect(self.show_fft)
        bar.addWidget(self.chk_fft)

        bar.addWidget(QtWidgets.QLabel("Span:"))
        self.cb_span = QtWidgets.QComboBox()
        self.cb_span.addItems([f"{s} kHz" for s in SPANS])
        self.cb_span.setCurrentIndex(1)
        self.cb_span.currentIndexChanged.connect(self.on_span)
        bar.addWidget(self.cb_span)
        bar.addStretch(1)

        # ---- plots (left) + side panel (right) ----
        body = QtWidgets.QHBoxLayout()
        layout.addLayout(body)

        left = QtWidgets.QVBoxLayout()
        body.addLayout(left, 1)
        self.pw = pw = pg.PlotWidget()
        left.addWidget(pw, 3)
        self.fw = fw = pg.PlotWidget()
        left.addWidget(fw, 2)
        body.addWidget(self.build_panel())

        pw.setMenuEnabled(False)
        pw.setMouseEnabled(False, False)
        pw.hideButtons()
        pw.setLabel("bottom", "Time (ms)")
        pw.setLabel("left", "Voltage (V)")

        grid_pen = pg.mkPen((60, 60, 60), width=1)
        mid_pen = pg.mkPen((120, 120, 120), width=1)
        self.vlines = []
        for i in range(11):
            ln = pg.InfiniteLine(angle=90, pen=mid_pen if i == 5 else grid_pen)
            pw.addItem(ln)
            self.vlines.append(ln)
        self.hlines = []
        for j in range(9):
            ln = pg.InfiniteLine(angle=0, pen=mid_pen if j == 4 else grid_pen)
            pw.addItem(ln)
            self.hlines.append(ln)

        trig_pen = pg.mkPen((255, 200, 0), width=1, style=QtCore.Qt.DashLine)
        self.trig_line = pg.InfiniteLine(pos=self.level, angle=0, movable=True,
                                         bounds=(0, 3.3), pen=trig_pen)
        self.trig_line.sigPositionChanged.connect(self.on_level_drag)
        pw.addItem(self.trig_line)

        cyan = (0, 220, 255)
        magenta = (255, 80, 255)
        self.cur_t = []
        for name in ("T1", "T2"):
            ln = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen(cyan, width=1),
                                 label=name,
                                 labelOpts={"position": 0.93, "color": cyan, "movable": False})
            ln.sigPositionChanged.connect(self.update_cursor_readout)
            pw.addItem(ln)
            self.cur_t.append(ln)
        self.cur_v = []
        for name in ("V1", "V2"):
            ln = pg.InfiniteLine(angle=0, movable=True, pen=pg.mkPen(magenta, width=1),
                                 label=name,
                                 labelOpts={"position": 0.97, "color": magenta, "movable": False})
            ln.sigPositionChanged.connect(self.update_cursor_readout)
            pw.addItem(ln)
            self.cur_v.append(ln)

        self.curve = pw.plot(pen=pg.mkPen((0, 255, 0), width=2))
        self.info = pg.TextItem(color=(0, 255, 0), anchor=(0, 0))
        pw.addItem(self.info)

        # ---- FFT plot ----
        fw.setMenuEnabled(False)
        fw.setMouseEnabled(False, False)
        fw.hideButtons()
        fw.showGrid(x=True, y=True, alpha=0.3)
        fw.setLabel("bottom", "Frequency (kHz)")
        fw.setLabel("left", "Level (dBV)")
        fw.setYRange(-90, 10, padding=0)
        self.fft_curve = fw.plot(pen=pg.mkPen((255, 170, 0), width=2),
                                 fillLevel=-90, fillBrush=(255, 170, 0, 50))
        self.fft_pts = pg.ScatterPlotItem(size=9, pen=pg.mkPen(None),
                                          brush=pg.mkBrush(255, 60, 60))
        fw.addItem(self.fft_pts)
        self.fft_info = pg.TextItem(color=(255, 170, 0), anchor=(0, 0))
        fw.addItem(self.fft_info)
        self.on_span()

        self.show_cursors(False)
        self.apply_scale()

        self.last_count = 0
        self.last_time = time.time()
        self.last_meas = 0.0
        self.last_fft = 0.0
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20)

    # ---- side panel ----
    def make_group(self, title, rows):
        box = QtWidgets.QGroupBox(title)
        grid = QtWidgets.QGridLayout(box)
        labels = {}
        for r, (name, key) in enumerate(rows):
            grid.addWidget(QtWidgets.QLabel(name), r, 0)
            lab = QtWidgets.QLabel("---")
            lab.setStyleSheet("background:#000; color:#0f0; font-family:Consolas;"
                              " font-size:14px; padding:2px 8px;")
            lab.setMinimumWidth(120)
            lab.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            grid.addWidget(lab, r, 1)
            labels[key] = lab
        return box, labels

    def build_panel(self):
        panel = QtWidgets.QWidget()
        col = QtWidgets.QVBoxLayout(panel)
        col.setContentsMargins(0, 0, 0, 0)
        box1, self.meas = self.make_group("Measurements", [
            ("Vmax", "vmax"), ("Vmin", "vmin"), ("Vpp", "vpp"),
            ("Vavg", "vavg"), ("Vrms", "vrms"), ("Vrms AC", "vac"),
            ("Freq", "freq"), ("Period", "period"), ("Duty", "duty"),
            ("THD (2-9)", "thd")])
        box2, self.curlab = self.make_group("Cursors", [
            ("T1", "t1"), ("T2", "t2"), ("dT", "dt"), ("1/dT", "inv"),
            ("V1", "v1"), ("V2", "v2"), ("dV", "dv"),
            ("V@T1", "vt1"), ("V@T2", "vt2")])
        col.addWidget(box1)
        col.addWidget(box2)
        col.addStretch(1)
        panel.setFixedWidth(250)
        return panel

    def update_measure(self):
        if self.last_v is None:
            return
        m = measure(self.last_v)

        self.period_samples = m["period"] * FS if m["period"] else None

        for key in ("vmax", "vmin", "vpp", "vavg", "vrms", "vac"):
            self.meas[key].setText(f"{m[key]:.2f} V")
        self.meas["freq"].setText(fmt_freq(m["freq"]))
        self.meas["period"].setText(fmt_period(m["period"]))
        self.meas["duty"].setText("---" if m["duty"] is None else f"{m['duty']:.1f} %")

    # ---- FFT ----
    def show_fft(self, flag):
        self.fft_on = bool(flag)
        self.fw.setVisible(self.fft_on)
        self.cb_span.setEnabled(self.fft_on)
        if not self.fft_on:
            self.meas["thd"].setText("---")

    def on_span(self, *args):
        self.span_khz = SPANS[self.cb_span.currentIndex()]
        self.fw.setXRange(0, self.span_khz, padding=0)
        self.fft_info.setPos(0.02 * self.span_khz, 8)

    def update_fft(self):
        v = self.last_v
        if v is None or not self.fft_on:
            return
        f, amp = spectrum(v)
        db = 20 * np.log10(np.maximum(amp, 1e-5))
        self.fft_curve.setData(f / 1000.0, db)
        df = f[1]

        lo = int(100 / df) + 1                 # ignore the DC region
        k = lo + int(np.argmax(amp[lo:]))
        a0 = amp[k]
        pts_x, pts_y = [], []
        thd = None
        if a0 > 0.05:
            delta = 0.0
            if 1 <= k < len(amp) - 1:
                a, b, c = db[k - 1], db[k], db[k + 1]
                d = a - 2 * b + c
                if d != 0:
                    delta = 0.5 * (a - c) / d
            f0 = (k + delta) * df
            self.fft_info.setText(f"Peak {fmt_freq(f0)}   {db[k]:.1f} dBV ({a0:.2f} V)")
            pts_x.append(f0 / 1000.0)
            pts_y.append(db[k])
            harm = []
            for h in range(2, 10):
                fh = h * f0
                if fh > FS / 2 - 500:
                    break
                i0 = max(int((fh - 100) / df), 0)
                i1 = int((fh + 100) / df) + 2
                seg = amp[i0:i1]
                if len(seg) == 0:
                    break
                j = i0 + int(np.argmax(seg))
                harm.append(amp[j])
                pts_x.append(f[j] / 1000.0)
                pts_y.append(db[j])
            if harm:
                thd = 100.0 * float(np.sqrt(np.sum(np.square(harm)))) / a0
        else:
            self.fft_info.setText("No signal")
        self.fft_pts.setData(pts_x, pts_y)
        self.meas["thd"].setText("---" if thd is None else f"{thd:.1f} %")

    # ---- cursors ----
    def show_cursors(self, flag):
        self.cursors_on = bool(flag)
        for ln in self.cur_t + self.cur_v:
            ln.setVisible(self.cursors_on)
        if self.cursors_on:
            td, vd, c = self.time_div, self.volt_div, self.center
            self.cur_t[0].setValue(3 * td)
            self.cur_t[1].setValue(7 * td)
            self.cur_v[0].setValue(c + 2 * vd)
            self.cur_v[1].setValue(c - 2 * vd)
            self.update_cursor_readout()
        else:
            for lab in self.curlab.values():
                lab.setText("---")

    def update_cursor_readout(self, *args):
        if not self.cursors_on:
            return
        t1, t2 = self.cur_t[0].value(), self.cur_t[1].value()
        v1, v2 = self.cur_v[0].value(), self.cur_v[1].value()
        dt = abs(t2 - t1)
        c = self.curlab
        c["t1"].setText(f"{t1:.3f} ms")
        c["t2"].setText(f"{t2:.3f} ms")
        c["dt"].setText(f"{dt:.3f} ms")
        c["inv"].setText(fmt_freq(1000.0 / dt) if dt > 1e-6 else "---")
        c["v1"].setText(f"{v1:.2f} V")
        c["v2"].setText(f"{v2:.2f} V")
        c["dv"].setText(f"{abs(v2 - v1):.2f} V")
        if self.disp is not None and len(self.disp) > 1:
            x = self.t[:len(self.disp)]
            for key, tt in (("vt1", t1), ("vt2", t2)):
                if x[0] <= tt <= x[-1]:
                    c[key].setText(f"{np.interp(tt, x, self.disp):.2f} V")
                else:
                    c[key].setText("---")

    # ---- display helpers ----
    def apply_scale(self):
        td, vd, c = self.time_div, self.volt_div, self.center
        self.W = int(round(td * 10 * FS / 1000))
        self.t = np.arange(self.W) / FS * 1000
        self.pw.setXRange(0, 10 * td, padding=0)
        self.pw.setYRange(c - 4 * vd, c + 4 * vd, padding=0)
        for i, ln in enumerate(self.vlines):
            ln.setPos(i * td)
        for j, ln in enumerate(self.hlines):
            ln.setPos(c + (j - 4) * vd)
        for ln in self.cur_t:
            ln.setBounds((0, 10 * td))
            ln.setValue(min(max(ln.value(), 0), 10 * td))
        for ln in self.cur_v:
            ln.setBounds((c - 4 * vd, c + 4 * vd))
            ln.setValue(min(max(ln.value(), c - 4 * vd), c + 4 * vd))
        bx = [(i * td, f"{round(i * td, 3):g}") for i in range(11)]
        ly = [(c + (j - 4) * vd, f"{round(c + (j - 4) * vd, 2):g}") for j in range(9)]
        self.pw.getAxis("bottom").setTicks([bx])
        self.pw.getAxis("left").setTicks([ly])
        self.update_info()
        self.redraw()

    def update_info(self):
        edge = "rising" if self.rising else "falling"
        self.info.setText(f"{FS / 1000:.0f} kS/s   {self.time_div} ms/div   "
                          f"{self.volt_div} V/div   Trig {self.level:.2f} V {edge}")
        self.info.setPos(0.1 * self.time_div,
                         self.center + 4 * self.volt_div - 0.05 * self.volt_div)

    def redraw(self):
        v = self.last_v
        if v is None:
            return
        i = find_trigger(v, self.level, self.rising)
        if self.scroll_on:
            room = len(v) - self.W - i
            wrap = self.period_samples if self.period_samples else 100
            wrap = max(1, min(int(wrap), room))
            i += int(self.phase) % wrap
        seg = v[i:i + self.W]
        self.disp = seg
        self.curve.setData(self.t[:len(seg)], seg)



    # ---- control handlers ----
    def toggle_run(self):
        self.run = not self.run
        self.btn_run.setText("Stop" if self.run else "Run")

    def on_edge(self, idx):
        self.rising = (idx == 0)
        self.update_info()
        self.redraw()

    def on_level_drag(self):
        self.set_level(self.trig_line.value())

    def set_level(self, val):
        self.level = float(val)
        self.sp_level.blockSignals(True)
        self.sp_level.setValue(self.level)
        self.sp_level.blockSignals(False)
        self.trig_line.blockSignals(True)
        self.trig_line.setValue(self.level)
        self.trig_line.blockSignals(False)
        self.update_info()
        self.redraw()

    def on_scale(self, *args):
        self.time_div = TIME_DIVS[self.cb_time.currentIndex()]
        self.volt_div = VOLT_DIVS[self.cb_volt.currentIndex()]
        self.center = self.sp_center.value()
        self.apply_scale()

    # ---- live update ----
    def update_plot(self):
        v = self.reader.latest
        if v is not None and self.run:
            self.last_v = v
            if self.scroll_on:
                self.phase += self.scroll_speed
            self.redraw()
        now = time.time()
        if now - self.last_fft >= 0.1:
            self.update_fft()
            self.last_fft = now
        if now - self.last_meas >= 0.25:
            self.update_measure()
            self.update_cursor_readout()
            self.last_meas = now
        if now - self.last_time >= 1.0:
            fps = (self.reader.count - self.last_count) / (now - self.last_time)
            self.statusBar().showMessage(f"{PORT}   |   {fps:.0f} frames/s")
            self.last_count = self.reader.count
            self.last_time = now

    def closeEvent(self, event):
        self.reader.running = False
        event.accept()


if __name__ == "__main__":
    try:
        reader = Reader(PORT)
    except serial.SerialException as e:
        print("Could not open", PORT, "-", e)
        print("Close PuTTY or any other serial program, and check the COM number.")
        sys.exit(1)
    reader.start()

    app = QtWidgets.QApplication(sys.argv)
    win = Scope(reader)
    win.show()
    code = app.exec_()
    reader.running = False
    sys.exit(code)