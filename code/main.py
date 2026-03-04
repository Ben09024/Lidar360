import sys, serial, struct, threading, time, math
import serial.tools.list_ports
import numpy as np
from PyQt6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

BAUDRATE = 115200
MAX_SPEED = 20000
POINT_LIFETIME = 12.0

STYLE = """
QMainWindow { background-color: #0b0c10; }
QWidget { font-family: 'Segoe UI', sans-serif; font-size: 14px; color: #c5c6c7; }
QGroupBox { background-color: #1f2833; border: 1px solid #45a29e; border-radius: 5px; margin-top: 15px; padding: 15px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; color: #66fcf1; font-weight: bold; text-transform: uppercase; letter-spacing: 1px; }
QPushButton { background-color: #1f2833; border: 1px solid #45a29e; border-radius: 3px; padding: 8px; color: #66fcf1; font-weight: bold; }
QPushButton:hover { background-color: #45a29e; color: #0b0c10; }
QPushButton#btn_con { background-color: #1f2833; border-color: #66fcf1; }
QPushButton#btn_start { color: #66fcf1; }
QPushButton#btn_stop { color: #ff0055; border-color: #ff0055; }
QPushButton#btn_stop:hover { background-color: #ff0055; color: white; }
QComboBox { background-color: #0b0c10; border: 1px solid #45a29e; padding: 5px; color: #66fcf1; }
QLabel#stat { font-size: 18px; font-weight: bold; color: #66fcf1; font-family: 'Consolas'; }
QSlider::groove:horizontal { height: 6px; background: #0b0c10; border: 1px solid #45a29e; }
QSlider::handle:horizontal { background: #66fcf1; width: 14px; margin: -5px 0; border-radius: 7px; }
"""

def calc_crc8(data):
    crc = 0
    for byte in data:
        extract = byte
        for _ in range(8):
            sum_val = (crc ^ extract) & 0x01
            crc >>= 1
            if sum_val: crc ^= 0x8C
            extract >>= 1
    return crc

class SerialWorker(QtCore.QThread):
    data_received = QtCore.pyqtSignal(list)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, BAUDRATE, timeout=0.05)
            PACKET_SIZE = 12
            UNPACK = '<BBfHHBB'
            buffer = bytearray()
            points_batch = []

            while self.running:
                if self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting)
                    buffer.extend(chunk)
                    if len(buffer) > 4000: buffer = buffer[-2000:]

                    while len(buffer) >= PACKET_SIZE:
                        if buffer[0] != 0xAA or buffer[1] != 0xBB:
                            del buffer[0]
                            continue

                        pkt = buffer[:PACKET_SIZE]
                        if calc_crc8(pkt[2:10]) != pkt[10]:
                            del buffer[:PACKET_SIZE]
                            continue

                        data = struct.unpack(UNPACK, pkt)
                        del buffer[:PACKET_SIZE]
                        angle, dist, strength = data[2:5]

                        if 20 < dist < 12000:
                            rad = math.radians(angle)
                            x = dist * math.cos(rad)
                            y = dist * math.sin(rad)
                            t = time.time()
                            points_batch.append([x, y, dist, t])

                if len(points_batch) > 50:
                    self.data_received.emit(points_batch)
                    points_batch = []
                time.sleep(0.0005)

        except Exception as e:
            print(f"Serial Error: {e}")
        finally:
            if self.ser: self.ser.close()

    def send(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(f"{cmd}\n".encode())
            except:
                pass

    def stop(self):
        self.running = False
        self.wait()

class LidarDashboard(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Lidar Core")
        self.resize(1400, 900)
        self.setStyleSheet(STYLE)
        self.worker = None
        self.data_store = np.empty((0, 4))
        self.init_ui()

    def init_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        sidebar = QtWidgets.QWidget()
        sidebar.setFixedWidth(300)
        sidebar.setStyleSheet("background-color: #12161a; border-right: 1px solid #45a29e;")
        vbox = QtWidgets.QVBoxLayout(sidebar)
        vbox.setContentsMargins(20, 20, 20, 20)

        gb_con = QtWidgets.QGroupBox("System Link")
        l_con = QtWidgets.QVBoxLayout()
        self.cb_ports = QtWidgets.QComboBox()
        self.refresh_ports()
        h_port = QtWidgets.QHBoxLayout()
        btn_refresh = QtWidgets.QPushButton("↻")
        btn_refresh.setFixedWidth(40)
        btn_refresh.clicked.connect(self.refresh_ports)
        h_port.addWidget(self.cb_ports)
        h_port.addWidget(btn_refresh)
        self.btn_con = QtWidgets.QPushButton("VERBINDEN")
        self.btn_con.setObjectName("btn_con")
        self.btn_con.clicked.connect(self.toggle_connect)
        l_con.addLayout(h_port)
        l_con.addWidget(self.btn_con)
        gb_con.setLayout(l_con)

        gb_ctl = QtWidgets.QGroupBox("Scanner Control")
        l_ctl = QtWidgets.QVBoxLayout()
        self.btn_start = QtWidgets.QPushButton("INITIATE SCAN")
        self.btn_start.setObjectName("btn_start")
        self.btn_start.clicked.connect(self.start_smooth)
        self.btn_stop = QtWidgets.QPushButton("EMERGENCY STOP")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.clicked.connect(self.stop_motor)
        self.lbl_speed = QtWidgets.QLabel("RPM: 600")
        self.sld_speed = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.sld_speed.setRange(200, MAX_SPEED)
        self.sld_speed.setValue(600)
        self.sld_speed.valueChanged.connect(self.change_speed_live)
        btn_clear = QtWidgets.QPushButton("CLEAR BUFFER")
        btn_clear.clicked.connect(self.clear_cloud)
        l_ctl.addWidget(self.btn_start)
        l_ctl.addWidget(self.btn_stop)
        l_ctl.addSpacing(20)
        l_ctl.addWidget(self.lbl_speed)
        l_ctl.addWidget(self.sld_speed)
        l_ctl.addSpacing(20)
        l_ctl.addWidget(btn_clear)
        gb_ctl.setLayout(l_ctl)

        gb_stat = QtWidgets.QGroupBox("Live Metrics")
        l_stat = QtWidgets.QFormLayout()
        self.lbl_count = QtWidgets.QLabel("0")
        self.lbl_count.setObjectName("stat")
        self.lbl_state = QtWidgets.QLabel("STANDBY")
        self.lbl_state.setObjectName("stat")
        self.lbl_state.setStyleSheet("color: #888;")
        l_stat.addRow("Points:", self.lbl_count)
        l_stat.addRow("Status:", self.lbl_state)
        gb_stat.setLayout(l_stat)

        vbox.addWidget(gb_con)
        vbox.addWidget(gb_ctl)
        vbox.addWidget(gb_stat)
        vbox.addStretch()

        self.plot_widget = pg.PlotWidget(background='#0b0c10')
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.getPlotItem().getAxis('bottom').setPen('#45a29e')
        self.plot_widget.getPlotItem().getAxis('left').setPen('#45a29e')
        self.plot_widget.addItem(pg.ScatterPlotItem(x=[0], y=[0], pen=None, brush='#ff0055', symbol='+', size=20))
        self.scatter_item = pg.ScatterPlotItem(size=3, pxMode=True)
        self.plot_widget.addItem(self.scatter_item)

        layout.addWidget(sidebar)
        layout.addWidget(self.plot_widget)

    def refresh_ports(self):
        self.cb_ports.clear()
        self.cb_ports.addItems([p.device for p in serial.tools.list_ports.comports()])

    def toggle_connect(self):
        if self.worker is None:
            port = self.cb_ports.currentText()
            if not port: return
            self.worker = SerialWorker(port)
            self.worker.data_received.connect(self.handle_data)
            self.worker.start()
            self.btn_con.setText("TRENNEN")
            self.btn_con.setStyleSheet("color: #ff0055; border-color: #ff0055;")
            self.lbl_state.setText("ONLINE")
            self.lbl_state.setStyleSheet("color: #66fcf1;")
        else:
            self.worker.stop()
            self.worker = None
            self.btn_con.setText("VERBINDEN")
            self.btn_con.setStyleSheet("")
            self.lbl_state.setText("STANDBY")
            self.lbl_state.setStyleSheet("color: #888;")

    def send(self, cmd):
        if self.worker: self.worker.send(cmd)

    def start_smooth(self):
        if not self.worker: return
        self.send("M100")
        self.lbl_state.setText("SCANNING")
        self.lbl_state.setStyleSheet("color: #00ff00;")
        target_speed = self.sld_speed.value()
        threading.Thread(target=self._ramp_thread, args=(target_speed,), daemon=True).start()

    def _ramp_thread(self, target):
        start_val = 200
        step = 100
        delay = 0.02
        current = start_val
        while current < target:
            self.send(f"M203 {current}")
            current += step
            time.sleep(delay)
        self.send(f"M203 {target}")

    def stop_motor(self):
        self.send("M101")
        self.lbl_state.setText("HALTED")
        self.lbl_state.setStyleSheet("color: #ff0055;")

    def change_speed_live(self, val):
        self.lbl_speed.setText(f"RPM: {val}")
        if self.worker:
            self.send(f"M203 {val}")

    def clear_cloud(self):
        self.data_store = np.empty((0, 4))
        self.scatter_item.clear()

    def handle_data(self, batch):
        if not batch: return
        new_pts = np.array(batch)
        self.data_store = np.vstack((self.data_store, new_pts))

    def update_visualization(self):
        if len(self.data_store) == 0: return
        now = time.time()
        mask = self.data_store[:, 3] > (now - POINT_LIFETIME)
        self.data_store = self.data_store[mask]

        if len(self.data_store) == 0:
            self.scatter_item.clear()
            self.lbl_count.setText("0")
            return

        age = now - self.data_store[:, 3]
        norm_age = np.clip(age / POINT_LIFETIME, 0, 1)
        alpha = (255 * (1.0 - norm_age)).astype(np.ubyte)
        n = len(alpha)
        colors = np.zeros((n, 4), dtype=np.ubyte)
        colors[:, 0] = (102 + (153 * norm_age)).astype(np.ubyte)
        colors[:, 1] = (252 * (1.0 - norm_age)).astype(np.ubyte)
        colors[:, 2] = 241
        colors[:, 3] = alpha

        self.scatter_item.setData(
            x=self.data_store[:, 0],
            y=self.data_store[:, 1],
            brush=colors,
            pen=None
        )
        self.lbl_count.setText(str(len(self.data_store)))

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = LidarDashboard()
    timer = QtCore.QTimer()
    timer.timeout.connect(window.update_visualization)
    timer.start(30)
    window.show()
    sys.exit(app.exec())
