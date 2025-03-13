import sys
import time
import socket
from PySide6.QtGui import QPixmap, QPainter, QColor, QFont, QBrush, QPalette
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QLineEdit,
                               QVBoxLayout, QComboBox, QPushButton, QTableWidget, 
                               QTableWidgetItem, QTabWidget, QLabel, QHBoxLayout)
import pyqtgraph as pg
from pathlib import Path
import datetime
import os


def connect_to_esp32_tcp(esp32_ssid, port=3333):
    """
    Create a TCP socket to the ESP32 server. If successful, return the socket.
    """
    if esp32_ssid == "MyESP32AP":
        esp32_ip = "192.168.4.1"
    else:
        esp32_ip = "192.168.4.2"

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)  # 2-second timeout for connect
    s.connect((esp32_ip, port))
    s.settimeout(0.0)  # Non-blocking mode for later reads
    print(f"[INFO] Connected to ESP32 at {esp32_ip}:{port}")
    return s

def parse_to_dict(message: str):
    """
    Parse a "key:value,key:value" message into a Python dictionary.
    """
    result = {}
    pairs = message.split(',')
    for pair in pairs:
        if ':' in pair:
            key, val = pair.split(':', 1)
            key = key.strip()
            val = val.strip()
            result[key] = val
    return result

###############################################################################
# MainWindow with GUI
###############################################################################
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Socket
        self.setWindowTitle("ESP32 Data Viewer")
        self.sock = None  # Will hold our TCP socket once connected
        self.read_buffer = ""  # Buffer for partial reads from socket
        self.socket_timer_interupt = 200

        # Calibration vars
        SRC_DIR = os.path.dirname(os.path.abspath(str(sys.modules['__main__'].__file__)))
        APPLICATION_DIR = Path(SRC_DIR).parent.absolute()
        self.cal_pwm = False
        self.cal_pwm_file_path = os.path.join(APPLICATION_DIR, "PWM_calibration.txt")
        self.cal_pwm_values = {}

        # We’ll track data for plotting
        self.plot_x = []
        self.plot_y = []
        self.timeSinceStart = 0
        self.previous_graph_key = None
        self.data = {}
        self.previous_map_pts = []
        self.counter = 0                            # increments each time we get new data
        self.start_state = "stop"
        self.connect_state = "disconnect"

        self.current_tot = {}
        self.current_min = {}
        self.current_max = {}

        # -----------------------
        # Central Widget + Layout
        # -----------------------
        self.main_window_minHeight = 600
        self.main_window_minWidth = 800

        # Central Widget and main window
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.setMinimumSize(self.main_window_minWidth, self.main_window_minHeight)

        # Layouts
        main_layout = QVBoxLayout(central_widget)
        get_pid_layout = QHBoxLayout()
        set_pid_layout = QHBoxLayout()
        set_pwm_layout = QHBoxLayout()
        start_cal_layout= QVBoxLayout()
        group_layout_1 = QVBoxLayout()
        group_layout_2 = QVBoxLayout()
        mainButton_layout = QHBoxLayout()

        # Dropdown
        self.dropdown = QComboBox()
        self.dropdown.setFixedSize(316, 30)

        # Pre-populate with the AP SSID we want
        self.dropdown.addItem("MyESP32AP")  
        self.dropdown.addItem("OtherAP")    # example extra
        main_layout.addWidget(self.dropdown)

        # Connect Button
        self.connect_button = QPushButton("Connect")
        self.connect_button.setFixedSize(316, 30)
        self.connect_button.clicked.connect(self.handle_connect_clicked)
        main_layout.addWidget(self.connect_button, stretch=0, alignment=Qt.AlignLeft)

        # Start Button
        self.start_button = QPushButton("Start")
        self.start_button.setFixedSize(100, 30)
        self.start_button.clicked.connect(self.handle_start_clicked)
        start_cal_layout.addWidget(self.start_button, stretch=1, alignment=Qt.AlignLeft)

        # Calibrate Button
        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.setFixedSize(100, 30)
        self.calibrate_button.clicked.connect(self.handle_calibrate_clicked)
        start_cal_layout.addWidget(self.calibrate_button, stretch=1, alignment=Qt.AlignLeft)

        # Set PWM button
        self.set_pwm_button = QPushButton("Set PWM")
        self.set_pwm_button.setFixedSize(100, 30)
        self.set_pwm_button.clicked.connect(self.handle_set_pwm_clicked)
        self.set_pwm_field = QLineEdit()
        self.set_pwm_field.setPlaceholderText("PWM")
        self.set_pwm_field.setFixedSize(100, 30)
        set_pwm_layout.addWidget(self.set_pwm_button, stretch=1, alignment=Qt.AlignLeft)
        set_pwm_layout.addWidget(self.set_pwm_field, stretch=100, alignment=Qt.AlignLeft)

        # Get PID Button
        self.get_pid_button = QPushButton("Get PID")
        self.get_pid_button.setFixedSize(100, 30)
        self.get_pid_button.clicked.connect(self.handle_get_pid_clicked)
        self.get_pid_label = QLabel("PID parameters:")
        get_pid_layout.addWidget(self.get_pid_button, stretch=1, alignment=Qt.AlignLeft)
        get_pid_layout.addWidget(self.get_pid_label, stretch=100, alignment=Qt.AlignLeft)

        # Set PID Button
        self.set_pid_button = QPushButton("Set PID")
        self.set_pid_button.setFixedSize(100, 30)
        self.set_pid_button.clicked.connect(self.handle_set_pid_clicked)
        self.set_pid_field = QLineEdit()
        self.set_pid_field.setPlaceholderText("type:Kp-Ki-Kd")
        self.set_pid_field.setFixedSize(100, 30)
        set_pid_layout.addWidget(self.set_pid_button, stretch=1, alignment=Qt.AlignLeft)
        set_pid_layout.addWidget(self.set_pid_field, stretch=100, alignment=Qt.AlignLeft)

        group_layout_1.addLayout(start_cal_layout)
        group_layout_2.addLayout(set_pwm_layout)
        group_layout_2.addLayout(set_pid_layout)
        group_layout_2.addLayout(get_pid_layout)

        mainButton_layout.addLayout(group_layout_1)
        mainButton_layout.addLayout(group_layout_2)

        main_layout.addLayout(mainButton_layout)

        # Tabs
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # --- Tab 1: Table of key-value ---
        self.tab1 = QWidget()
        self.tab1_layout = QVBoxLayout(self.tab1)
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(["Key", "Value", "Average", "Min", "Max"])
        # Start with 0 rows; we'll add rows dynamically
        self.table.setRowCount(0)

        self.tab1_layout.addWidget(self.table)
        self.tabs.addTab(self.tab1, "Table")

        # --- Tab 2: Graph (using pyqtgraph) ---
        self.tab2 = QWidget()
        self.tab2_layout = QVBoxLayout(self.tab2)
        self.dropdownGraphValue = QComboBox()
        self.dropdownGraphValue.addItem("Select a value")
        self.tab2_layout.addWidget(self.dropdownGraphValue)

        self.graph_widget = pg.PlotWidget()
        self.plot_item = self.graph_widget.getPlotItem()
        self.plot_item.showGrid(x=True, y=True)
        self.legendItem = pg.LegendItem()
        self.tab2_layout.addWidget(self.graph_widget)

        # self.tabs.addTab(self.tab2, "Graph")

        # --- Tab 3: Canvas for drawing maps ---
        self.tab3 = QWidget()
        self.tab3_layout = QVBoxLayout(self.tab3)
        self.draw_canvas = QWidget()
        self.label = QLabel(self.draw_canvas)
        self.tab3_layout.addWidget(self.draw_canvas)
        # self.tabs.addTab(self.tab3, "Map")
        # self.init_map()

        # Create a timer to poll the socket for incoming data
        print("Create timer")
        self.socket_timer = QTimer()
        self.socket_timer.setInterval(self.socket_timer_interupt)
        self.socket_timer.timeout.connect(self.read_from_socket)

    def clear_map(self):
        self.label.clear()

    def init_map(self, width=1400, height=600):
        # Set map size and initial background color
        self.canvas_size = (width, height)
        self.draw_canvas.width = width
        self.draw_canvas.height = height
        self.pixmap = QPixmap(self.draw_canvas.width, self.draw_canvas.height)
        self.pixmap.fill(Qt.white)  

        # Draw starting point
        start_width = 66
        start_height = 13
        self.start_x = start_width + 5
        self.start_y = start_height + 10
        self.position_text_x = width - 130
        self.position_text_y = 11
        self.map_painter = QPainter(self.pixmap)
        self.map_painter.setPen(QColor(0, 0, 0)) 
        self.map_painter.drawRect(0, 0, start_width, start_height)
        self.map_painter.drawLine(start_width*(3/4), start_height, self.start_x-2, self.start_y - 2.5)
        self.map_painter.drawLine(start_width, start_height, self.start_x-2, self.start_y - 2.5)
        self.map_painter.drawEllipse(self.start_x - 2.5, self.start_y - 2.5, 5, 5)
        self.map_painter.drawText(5, 11, "Start: (0, 0)")

        # Initialize position labels
        self.map_position_text = f"Position: (0, 0)"
        self.map_velocity_text = f"Velocity: 0"
        self.map_painter.drawText(self.position_text_x, self.position_text_y, self.map_position_text)
        self.map_painter.drawText(self.position_text_x, self.position_text_y + 15, self.map_velocity_text)


    def update_map_text(self, position = (0,0), velocity = 0):
        self.map_painter.setBrush(QBrush(QColor("white")))
        self.map_painter.setPen(Qt.NoPen)

        self.map_painter.drawRect(self.position_text_x - 10, 0, self.canvas_size[0] , 100)
        self.map_painter.setPen(QColor(0, 0, 0))
        self.map_position_text = f"Position: ({position[0]}, {position[1]})"
        self.map_velocity_text = f"Velocity: {velocity}" 
        self.map_painter.drawText(self.position_text_x, self.position_text_y, self.map_position_text)
        self.map_painter.drawText(self.position_text_x, self.position_text_y + 15, self.map_velocity_text)

    def draw_map(self, data, position, velocity):
        """
        Draw black pixels at specified (x, y) coordinates.
        :param data: List of (x, y) coordinates to be colored black.
        """
        for x, y in data:
            self.map_painter.drawPoint(x + self.start_x, y + self.start_y)

        self.update_map_text(position, velocity)
        self.label.setPixmap(self.pixmap)  # Update the display
        self.previous_map_pts = data

    def update_graph(self):
        colors = ['blue', 'orange', 'green', 'red', 'purple',
                'brown', 'gray', 'black', 'olive']
        # symbols = ['o', '+', 'x', 's', 't', 'd', 'p', 'h', 'star']
        title_text_style = {'color': 'white', 'size': '20pt'}

        if self.dropdownGraphValue.currentText() == "Select a value":
            self.dropdownGraphValue.setCurrentIndex(1)
            self.plot_x = []
            self.plot_y = []
            return

        self.current_graph_key = self.dropdownGraphValue.currentText()
        for index, (key, value) in enumerate(self.data.items()):
            if key == self.current_graph_key:
                if self.previous_graph_key != self.current_graph_key:
                    self.clear_graph()
                    self.plot_item.setTitle(f"{self.current_graph_key}", **title_text_style)
                    self.curve = self.plot_item.plot((self.plot_x[0], value), pen=colors[index])
                    self.legendItem.addItem(self.curve, self.current_graph_key)
                    self.plot_y.append(value)
                    self.previous_graph_key = self.current_graph_key
                else:
                    self.plot_y.append(value)
                    if len(self.plot_y) > 50:
                        self.curve.setData(self.plot_x[-20:], self.plot_y[-20:])
                    else:
                        self.curve.setData(self.plot_x, self.plot_y)

    def clear_graph(self):
        self.plot_item.clear()
        self.legendItem.clear()
        self.plot_y = []
        self.plot_x = [self.plot_x[-1]]

    def handle_set_pwm_clicked(self):
        print("Set PWM button clicked")
        pwm_name = self.set_pwm_field.text()
        self.cal_pwm = True
        with open(self.cal_pwm_file_path, "a") as file:
            file.write("**************************************************\n")
            file.write(f"PWM calibration started for: {pwm_name}\n")
            file.write("**************************************************\n")

        for pwm in range(0, 2058, 5):
            self.sock.sendall(f"setPwm:{pwm_name}:{pwm}".encode('utf-8'))
            print(f"Setting {pwm_name} PWM to: {pwm}")
            time.sleep(0.5)
            if self.cal_pwm_values['rpm'] > 0:
                current_pwm_val = pwm
                break
        for pwm in range(current_pwm_val, 0, -1):
            self.sock.sendall(f"setPwm:{pwm_name}:{pwm}".encode('utf-8'))
            print(f"Setting {pwm_name} PWM to: {pwm}")
            time.sleep(0.5)
            if self.cal_pwm_values['rpm'] <= 0:
                current_pwm_val = pwm
                current_stop_pwm = pwm
                break

        for pwm in range(current_pwm_val, 2058, 1):
            self.sock.sendall(f"setPwm:{pwm_name}:{pwm}".encode('utf-8'))
            print(f"Setting {pwm_name} PWM to: {pwm}")
            time.sleep(self.socket_timer_interupt/1000)
            if self.cal_pwm_values['rpm'] > 0:
                current_start_pwm = pwm
                break

        speed_inc = 0
        for pwm in range(current_start_pwm, 2058, 1):
            self.sock.sendall(f"setPwm:{pwm_name}:{pwm}".encode('utf-8'))
            print(f"Setting {pwm_name} PWM to: {pwm}")
            time.sleep(self.socket_timer_interupt/1000)
            previous_rpm = self.cal_pwm_values['rpm']
            time.sleep(self.socket_timer_interupt/1000)
            current_rpm = self.cal_pwm_values['rpm']
            if current_rpm > previous_rpm:
                speed_inc = 0
            else:
                speed_inc += 1
            if speed_inc > 10:
                current_max_rpm = current_rpm
                break

        with open(self.cal_pwm_file_path, "a") as file:
            file.write("**************************************************\n")
            file.write(f"{pwm_name} PWM: Found PWM value to make the motor stop, value: {current_stop_pwm}\n")
            file.write(f"{pwm_name} PWM: Found PWM value to make the motor move, value: {current_start_pwm}\n")
            file.write(f"{pwm_name} PWM: Found PWM value for max RPM, value: {current_max_rpm}\n")
            file.write("**************************************************\n")
            file.write("\n")
        
        self.cal_pwm = False

    def handle_start_clicked(self):
        if self.start_state == "stop":
            self.start_state = "start"
            self.sock.sendall(f"start".encode('utf-8'))
            self.start_button.setText("Stop")
        else:
            self.counter = 0
            self.start_state = "stop"
            self.sock.sendall(f"stop".encode('utf-8'))
            self.start_button.setText("Start")

    def handle_calibrate_clicked(self):
        print("Calibrate button clicked")
        self.sock.sendall(f"calibrate".encode('utf-8'))

    def handle_get_pid_clicked(self):
        print("Get PID button clicked")
        try:
            # Send the message to the ESP32
            self.sock.sendall(f"getPID".encode('utf-8'))
            time.sleep(0.01)
            # Wait for a response
            while True:
                data = self.sock.recv(1024)
                if data:
                    break
                time.sleep(0.01)  # Small delay to avoid high CPU usage
        except Exception as e:
            print(f"Error in send_and_receive: {e}")

        self.get_pid_label.setText(data.decode('utf-8', errors='ignore'))

    def handle_set_pid_clicked(self): 
        print("Set PID button clicked")
        pid_field = self.set_pid_field.text()
        pid_field = pid_field.split(":")
        pid_type = pid_field[0]
        pid_params = pid_field[1]
        self.sock.sendall(f"set{pid_type}PID:{pid_params}".encode('utf-8'))
        print(f"Setting {pid_type}PID parameters: {pid_params}")

    def handle_connect_clicked(self):
        if self.connect_state == "disconnect":
            self.connect_state = "connect"
            self.connect_button.setText("Disconnect")
            self.tabs.addTab(self.tab2, "Graph")
            self.tabs.addTab(self.tab3, "Map")
            self.init_map()

            ssid = self.dropdown.currentText()
            print(f"[INFO] Attempting to connect to SSID: {ssid}")
            port = 3333

            # 1) Create TCP socket to the ESP32 at 192.168.4.1:3333
            try:
                self.sock = connect_to_esp32_tcp(ssid)
                self.startTime = time.time()
                self.socket_timer.start()  # Start reading from the socket
            except Exception as e:
                print(f"[ERROR] Could not connect socket: {e}")
                self.sock = None
        else:
            self.connect_state = "disconnect"
            self.connect_button.setText("Connect")
            self.sock.close()
            self.sock = None
            self.socket_timer.stop()
            self.map_painter.end()
            self.tabs.setCurrentIndex(0)
            self.tabs.removeTab(self.tabs.indexOf(self.tab2))
            self.tabs.removeTab(self.tabs.indexOf(self.tab3))

    def set_annotation(self, text, pickX, pickY):
        self._pointAnnotation = pg.TextItem(text=text, color='black', anchor=(0,1), border=pg.mkPen("black"), fill=pg.mkBrush(200, 200, 200, 180))
        self._pointAnnotation.setFont(QFont("Arial", 14))
        self._pointHighlight = pg.TargetItem(pos=(pickX, pickY), size=20, symbol="crosshair", pen=pg.mkPen("black"), hoverPen=pg.mkPen("black"))

        # Add it to the plot
        self.plot_item.addItem(self._pointHighlight)
        self.plot_item.addItem(self._pointAnnotation)

        # Position at the data point
        self._pointAnnotation.setPos(pickX, pickY)

    def remove_annotation(self):
        if self._pointAnnotation is not None:
            self.plot_item.removeItem(self._pointAnnotation)
            self._pointAnnotation = None
        if self._pointHighlight is not None:
            self.plot_item.removeItem(self._pointHighlight)
            self._pointHighlight = None

    def read_from_socket(self):
        """
        Called periodically by QTimer to read available data from the socket (non-blocking).
        If there's a complete line, parse it, update table and graph.
        """
        if not self.sock:
            return

        # # Non-blocking read: attempt to receive up to 1024 bytes
        try:
            data = self.sock.recv(1024)
        except BlockingIOError:
            # No data available
            return
        except Exception as e:
            print(f"[ERROR] Socket read error: {e}")
            self.sock.close()
            self.sock = None
            self.socket_timer.stop()
            return

        if not data:
            # No data means connection closed by server
            print("[INFO] Connection closed by ESP32.")
            self.sock.close()
            self.sock = None
            self.socket_timer.stop()
            return

        # Decode the received bytes
        chunk = data.decode('utf-8', errors='ignore')
        self.read_buffer += chunk
        while "\n" in self.read_buffer:
            line, self.read_buffer = self.read_buffer.split("\n", 1)
            line = line.strip()
            if line:
                data_dict = self.process_incoming_line(line)
        self.read_buffer = ""

        if not self.cal_pwm:
            # Update the table
            self.update_table(data_dict)

            # Update the data dictionary
            current_time = time.time()
            self.timeSinceStart = current_time - self.startTime

            self.plot_x.append(self.timeSinceStart)
            for key, value in data_dict.items():
                self.data[key] = round(float(value), 3)

            x_pos = 0
            y_pos = 0
            velocity = 0
            map_pts = []
            for key, value in self.data.items():
                if key == "X position":
                    x_pos = value
                elif key == "Y position":
                    y_pos = value
                elif key == "Velocity":
                    velocity = value
                    
            map_pts.append((x_pos, y_pos))
            self.update_graph()
            self.draw_map(map_pts, (x_pos, y_pos), velocity)

        else:
            self.cal_pwm_values = data_dict
            self.update_table(data_dict)
            with open(self.cal_pwm_file_path, "a") as file:
                file.write(f"{data_dict['name']} PWM: {round(data_dict['PWM'], 2)}, RPM: {round(data_dict['RPM'], 2)}\n")

    def process_incoming_line(self, chunk: str=""):
        """
        Parse the line as "key:value,key:value", update the table and the graph.
        """
        # print(f"[INFO] Received line: {line}")

        # Parse to dict
        data_dict = parse_to_dict(chunk)
        
        self.counter += 1
        return data_dict

    def update_table(self, data_dict):
        """
        Replace the table contents with the items in data_dict.
        For each key-value pair, create/update a row.
        """
        # Save some current values
        for i, (k, v) in enumerate(data_dict.items()):
            if k != "message":
                if k not in self.current_tot:
                    self.current_tot[k] = 0
                    self.current_min[k] = 10000
                    self.current_max[k] = -10000
                self.current_tot[k] += v
                self.current_min[k] = min(self.current_min[k], v)
                self.current_max[k] = max(self.current_max[k], v)

        # Clear out existing rows
        self.table.setRowCount(0)
        self.dropdownGraphValue.setMaxCount(len(data_dict) + 1)
        self.message = ""

        # Populate rows from data_dict
        for i, (k, v) in enumerate(data_dict.items()):
            if k != "message":
                self.table.insertRow(i)
                self.table.setItem(i, 0, QTableWidgetItem(str(k)))
                self.table.setItem(i, 1, QTableWidgetItem(str(v)))
                self.table.setItem(i, 2, QTableWidgetItem(str(round((self.current_tot[k])/self.counter, 2))))
                self.table.setItem(i, 3, QTableWidgetItem(str(round(self.current_min[k], 2))))
                self.table.setItem(i, 4, QTableWidgetItem(str(round(self.current_max[k], 2))))
                self.dropdownGraphValue.addItem(str(k))
            else:
                self.message = v

        if self.message != "":
            if "parameters" in self.message:
                self.get_pid_label.setText(f"PID parameters: {self.message.split('-')[-1]}")
            self.message = ""

def apply_dark_mode(app):
    """Apply dark mode styling to a PySide6 application"""
    app.setStyle("Fusion")  # Use the Fusion style

    # Create a dark palette
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.Window, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Base, QColor(25, 25, 25))
    dark_palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ToolTipBase, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Text, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))

    app.setPalette(dark_palette)

def main():
    app = QApplication(sys.argv)
    apply_dark_mode(app)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()