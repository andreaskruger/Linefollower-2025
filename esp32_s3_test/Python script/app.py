from PySide6.QtGui import QPixmap, QPainter, QColor, QFont, QBrush, QPalette
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QLineEdit,
                               QVBoxLayout, QComboBox, QPushButton, QTableWidget, 
                               QTableWidgetItem, QTabWidget, QLabel, QHBoxLayout)
from pathlib import Path
import pyqtgraph as pg
import subprocess
import threading
import datetime
import socket
import time
import sys
import os


def connect_to_esp32_tcp(esp32_ssid, port=3333):
    """
    Create a TCP socket to the ESP32 server. If successful, return the socket.
    """
    if esp32_ssid == "SpeedyBoii" or "MyESP32AP":
        esp32_ip = "192.168.4.1"
        connect_to_network(esp32_ssid, "12345678")
    else:
        print(f"[ERROR] SSID {esp32_ssid} not found")
        return None

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)  # 2-second timeout for connect
    s.connect((esp32_ip, port))
    s.settimeout(0.0)  # Non-blocking mode for later reads
    print(f"[INFO] Connected to ESP32 at {esp32_ip}:{port}")
    return s

def connect_to_network(ssid, password):
    """
    Connect to the network using subprocess
    """
    profile_name = ssid  # For simplicity, we use the SSID as the profile name

    # Check if the profile already exists
    result = subprocess.run(
        ['netsh', 'wlan', 'show', 'profiles'],
        capture_output=True, text=True
    )

    if profile_name not in result.stdout:
        print(f"Profile for SSID '{ssid}' not found. Creating temporary profile...")

        # Create a Wi-Fi profile XML
        profile_xml = f'''<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
    <name>{ssid}</name>
    <SSIDConfig>
        <SSID>
            <name>{ssid}</name>
        </SSID>
    </SSIDConfig>
    <connectionType>ESS</connectionType>
    <connectionMode>manual</connectionMode>
    <MSM>
        <security>
            {"<authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption>" if password else "<authEncryption><authentication>open</authentication><encryption>none</encryption><useOneX>false</useOneX></authEncryption>"}
            {f"<sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>{password}</keyMaterial></sharedKey>" if password else ""}
        </security>
    </MSM>
</WLANProfile>'''

        # Save XML to file
        with open("temp_wifi_profile.xml", "w") as f:
            f.write(profile_xml)

        # Add profile to Windows
        subprocess.run(['netsh', 'wlan', 'add', 'profile', 'filename="temp_wifi_profile.xml"'], shell=True)

    # Connect to the network
    print(f"Connecting to '{ssid}'...")
    subprocess.run(['netsh', 'wlan', 'connect', f'name={profile_name}', f'ssid={ssid}'])

    # Wait until connected
    connected = False
    for i in range(10):
        result = subprocess.run(['netsh', 'wlan', 'show', 'interfaces'], capture_output=True, text=True)
        if ssid in result.stdout and "State" in result.stdout and "connected" in result.stdout.lower():
            connected = True
            print(f"Successfully connected to '{ssid}'!")
            break
        time.sleep(1)

    if not connected:
        print(f"Failed to connect to '{ssid}' within timeout.")
    return connected

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

        # -----------------------
        # Socket read buffers
        # -----------------------
        # self.latest_data = "description:data, X-position:0,Y-position:0,Velocity:0,Control signal left:0,Control signal right:0end\n"
        self.latest_data = None
        self.lock = threading.Lock()

        # -----------------------
        # Socket
        # -----------------------
        self.setWindowTitle("ESP32 Data Viewer")
        self.sock = None                    # Will hold our TCP socket once connected
        self.read_buffer = ""               # Buffer for partial reads from socket
        self.socket_timer_interupt = 20     # Timer for interupt for reading from socket
        self.waitingForRead = False

        # -----------------------
        # Calibration vars
        # -----------------------
        SRC_DIR = os.path.dirname(os.path.abspath(str(sys.modules['__main__'].__file__)))
        APPLICATION_DIR = Path(SRC_DIR).parent.absolute()
        self.cal_pwm = False
        self.cal_pwm_file_path = os.path.join(APPLICATION_DIR, "PWM_calibration.txt")
        self.cal_pwm_values = {}

        # -----------------------
        # Track data for plotting
        # -----------------------
        self.plot_x = []
        self.plot_y = []
        self.timeSinceStart = 0
        self.previous_graph_key = None
        self.graph_data = {}
        self.data = {}
        self.map_pts = []
        self.previous_map_pts = []
        self.counter = 0                            # Increments each time we get new data
        self.current_tot = {}
        self.current_min = {}
        self.current_max = {}

        self.start_state = "stop"
        self.connect_state = "disconnect"
        self.get_pid_state = False

        # -----------------------
        # Central Widget + Layout
        # -----------------------
        self.main_window_minHeight = 600
        self.main_window_minWidth = 800

        # Central Widget and main window
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.setMinimumSize(self.main_window_minWidth, self.main_window_minHeight)

        # -----------------------
        # Layouts
        # -----------------------
        main_layout = QVBoxLayout(central_widget)
        get_pid_layout = QHBoxLayout()
        set_pid_layout = QHBoxLayout()
        set_BS_layout = QHBoxLayout()
        start_cal_layout= QVBoxLayout()
        group_layout_1 = QVBoxLayout()
        group_layout_2 = QVBoxLayout()
        mainButton_layout = QHBoxLayout()

        # -----------------------
        # Add connection indicators
        # -----------------------
        self.connection_on_path = os.path.join(APPLICATION_DIR, "resources", "connected.jpg")
        self.connection_off_path = os.path.join(APPLICATION_DIR, "resources", "notConnected.jpg")
        self.connection_light_state = False
        self.connection_light = QLabel()
        self.connection_light.setPixmap(QPixmap(self.connection_off_path))
        self.connection_light.setScaledContents(True)
        self.connection_light.setFixedSize(30, 30)
        # main_layout.addWidget(self.connection_light, stretch=0, alignment=Qt.AlignLeft)
        
        # -----------------------
        # Dropdown
        # -----------------------
        self.dropdown = QComboBox()
        self.dropdown.setFixedSize(316, 30)
        self.dropdown.addItem("SpeedyBoii")      # Pre-populate with the AP SSID we want
        # self.dropdown.addItem("MyESP32AP") 
        main_layout.addWidget(self.dropdown)

        # -----------------------
        # Connect Buttons
        # -----------------------
        self.connect_button = QPushButton("Connect")
        self.connect_button.setFixedSize(316, 30)
        self.connect_button.clicked.connect(self.handle_connect_clicked)
        main_layout.addWidget(self.connect_button, stretch=0, alignment=Qt.AlignLeft)

        # -----------------------
        # Add State indicator
        # -----------------------
        self.robot_state_label = QLabel("Robot state: ")
        self.robot_state_label.setFixedSize(100, 30)
        main_layout.addWidget(self.robot_state_label, stretch=100, alignment=Qt.AlignLeft)

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

        # Test PWM Button
        self.PWM_test_button = QPushButton("PWM")
        self.PWM_test_button.setFixedSize(100, 30)
        self.PWM_test_button.clicked.connect(self.handle_set_pwm_clicked)
        start_cal_layout.addWidget(self.PWM_test_button, stretch=1, alignment=Qt.AlignLeft)

        # Set base speed button
        self.set_BS_button = QPushButton("Set base speed")
        self.set_BS_button.setFixedSize(100, 30)
        self.set_BS_button.clicked.connect(self.handle_set_BS_clicked)
        self.set_BS_field = QLineEdit()
        self.set_BS_field.setPlaceholderText("Base speed")
        self.set_BS_field.setFixedSize(100, 30)
        set_BS_layout.addWidget(self.set_BS_button, stretch=1, alignment=Qt.AlignLeft)
        set_BS_layout.addWidget(self.set_BS_field, stretch=100, alignment=Qt.AlignLeft)

        # Get PID Button
        self.get_pid_button = QPushButton("Get PID")
        self.get_pid_button.setFixedSize(100, 30)
        self.get_pid_button.clicked.connect(self.handle_get_pid_clicked)
        self.get_pid_label = QLabel("PID parameters: ")
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
        group_layout_2.addLayout(set_BS_layout)
        group_layout_2.addLayout(set_pid_layout)
        group_layout_2.addLayout(get_pid_layout)

        mainButton_layout.addLayout(group_layout_1)
        mainButton_layout.addLayout(group_layout_2)

        main_layout.addLayout(mainButton_layout)

        # -----------------------
        # Tabs
        # -----------------------
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # --- Tab 1: Table of key-value ---
        self.tab1 = QWidget()
        self.tab1_layout = QVBoxLayout(self.tab1)
        self.table = QTableWidget()
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(["Key", "Value", "Average", "Min", "Max"])
        # Start with 0 rows; we'll add rows dynamically
        self.table.setRowCount(0)

        self.tab1_layout.addWidget(self.table)
        self.tabs.addTab(self.tab1, "Table")

        # --- Tab 2: Graph (using pyqtgraph) ---
        self.tab2 = QWidget()
        self.tab2_layout = QVBoxLayout(self.tab2)
        self.dropdownGraphValue = QComboBox()
        self.dropdownGraphValue.setMaxCount(1)
        self.dropdownGraphValue.addItem("Select a value")
        self.tab2_layout.addWidget(self.dropdownGraphValue)

        self.graph_widget = pg.PlotWidget()
        self.plot_item = self.graph_widget.getPlotItem()
        self.plot_item.showGrid(x=True, y=True)
        self.legendItem = pg.LegendItem()
        self.tab2_layout.addWidget(self.graph_widget)

        # --- Tab 3: Canvas for drawing maps ---
        self.tab3 = QWidget()
        self.tab3_layout = QVBoxLayout(self.tab3)
        self.draw_canvas = QWidget()
        self.label = QLabel(self.draw_canvas)
        self.tab3_layout.addWidget(self.draw_canvas)

        # -----------------------
        # Setup update GUI timer
        # -----------------------
        print("Create timer")
        self.socket_timer = QTimer()
        self.socket_timer.setInterval(self.socket_timer_interupt)
        self.socket_timer.timeout.connect(self.update_rec_data)


    def update_connection_light(self, state):
        if state:
            self.connection_light.setPixmap(QPixmap(self.connection_on_path))
        else:
            self.connection_light.setPixmap(QPixmap(self.connection_off_path))

    def update_state_label(self, state):
        self.robot_state_label.setText(f"Robot state: {state}")    
        if "start" in state:
            self.start_button.setText("Stop")
            self.start_state = "start"
        elif "stop" in state:
            self.start_state = "stop"
            self.start_button.setText("Start")
        elif "Finish line found" in state:
            self.start_button.setText("Start")
            self.start_state = "stop"

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
        self.map_painter.drawText(5, 11, "Start: (0, 0)")

        #--------Draw the start point---------#
        # self.map_painter.drawLine(start_width*(3/4), start_height, self.start_x-2, self.start_y - 2.5)
        # self.map_painter.drawLine(start_width, start_height, self.start_x-2, self.start_y - 2.5)
        # self.map_painter.drawEllipse(self.start_x - 2.5, self.start_y - 2.5, 5, 5)

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

    def format_graph_data(self, data_dict):
        keys = list(data_dict.keys())
        values = list(data_dict.values())
        nr_of_keys = len(keys)
        current_time = time.time()
        self.timeSinceStart = current_time - self.startTime

        self.plot_x.append(self.timeSinceStart)
        for index, (key, value) in enumerate(data_dict.items()):
            if key != "description":
                if key not in self.graph_data:
                    self.graph_data[key] = [round(float(value), 3)]
                else:
                    self.graph_data[key].append(round(float(value), 3))

    def update_graph(self, data_dict):
        self.format_graph_data(data_dict)
        colors = ['blue', 'orange', 'green', 'red', 'purple',
                'brown', 'gray', 'black', 'olive']
        # symbols = ['o', '+', 'x', 's', 't', 'd', 'p', 'h', 'star']
        title_text_style = {'color': 'white', 'size': '20pt'}

        self.current_graph_key = self.dropdownGraphValue.currentText()
        if self.current_graph_key == "Select a value":
            return

        for index, (key, value) in enumerate(self.graph_data.items()):
            if key != "description":
                if key == self.current_graph_key:
                    if self.previous_graph_key != self.current_graph_key:
                        self.clear_graph()
                        self.plot_item.setTitle(f"{self.current_graph_key}", **title_text_style)
                        if len(self.graph_data[key]) > 50:
                            self.curve = self.plot_item.plot(self.plot_x[-50:], self.graph_data[key][-50:], pen=colors[index])
                        else:
                            self.curve = self.plot_item.plot(self.plot_x, self.graph_data[key], pen=colors[index])
                        
                        self.legendItem.addItem(self.curve, self.current_graph_key)
                        # self.plot_y.append(float(value))
                        self.previous_graph_key = self.current_graph_key
                    else:
                        # self.plot_y.append(value)
                        if len(self.graph_data[key]) > 50:
                            self.curve.setData(self.plot_x[-50:], self.graph_data[key][-50:])
                        else:
                            self.curve.setData(self.plot_x, self.graph_data[key])


    def clear_graph(self):
        self.plot_item.clear()
        self.legendItem.clear()

    def handle_set_BS_clicked(self):
        print("Set base speed button clicked")
        base_speed = self.set_BS_field.text()
        self.sock.sendall(f"setBaseSpeed:{base_speed}".encode('utf-8'))

    def handle_set_pwm_clicked(self):
        print("Set PWM button clicked")
        self.cal_pwm = True
        for i in range(2):
            description = "setLeftPwm" if i == 0 else "setRightPwm"
            with open(self.cal_pwm_file_path, "a") as file:
                file.write("**************************************************\n")
                file.write(f"PWM calibration started for {description}.\n")
                file.write("**************************************************\n")

            for pwm in range(0, 2058, 5):
                self.sock.sendall(f"{description}:{pwm}".encode('utf-8'))
                print(f"Setting {description[2:]} to: {pwm}")
                time.sleep(0.5)
                if self.cal_pwm_values['rpm'] > 0:
                    current_pwm_val = pwm
                    break
            for pwm in range(current_pwm_val, 0, -1):
                self.sock.sendall(f"{description}:{pwm}".encode('utf-8'))
                print(f"Setting {description[2:]} to: {pwm}")
                time.sleep(0.5)
                if self.cal_pwm_values['rpm'] <= 0:
                    current_pwm_val = pwm
                    current_stop_pwm = pwm
                    break

            for pwm in range(current_pwm_val, 2058, 1):
                self.sock.sendall(f"{description}::{pwm}".encode('utf-8'))
                print(f"Setting {description[2:]} to: {pwm}")
                time.sleep(self.socket_timer_interupt/1000)
                if self.cal_pwm_values['rpm'] > 0:
                    current_start_pwm = pwm
                    break

            speed_inc = 0
            for pwm in range(current_start_pwm, 2058, 1):
                self.sock.sendall(f"{description}:{pwm}".encode('utf-8'))
                print(f"Setting {description[2:]} to: {pwm}")
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

            print(f"Setting {description[2:]} to: 0")
            self.sock.sendall(f"{description}:0".encode('utf-8'))
            with open(self.cal_pwm_file_path, "a") as file:
                file.write("**************************************************\n")
                file.write(f"{description[2:]}: Found PWM value to make the motor stop, value: {current_stop_pwm}\n")
                file.write(f"{description[2:]}: Found PWM value to make the motor move, value: {current_start_pwm}\n")
                file.write(f"{description[2:]}: Found PWM value for max RPM, value: {current_max_rpm}\n")
                file.write("**************************************************\n")
                file.write("\n")
        
        self.cal_pwm = False

    def handle_start_clicked(self):
        if self.start_state == "stop":
            print("Start button clicked")
            self.sock.sendall(f"start".encode('utf-8'))
            self.start_state = "start"
            self.start_button.setText("Stop")
        else:
            print("Stop button clicked")
            self.sock.sendall(f"stop".encode('utf-8'))
            self.start_state = "stop"
            self.start_button.setText("Start")

    def handle_calibrate_clicked(self):
        print("Calibrate button clicked")
        self.sock.sendall(f"calibrate".encode('utf-8'))

    def handle_get_pid_clicked(self):
        print("Get PID button clicked")
        self.get_pid_state = True
        self.sock.sendall(f"getPID".encode('utf-8'))

    def handle_set_pid_clicked(self): 
        print("Set PID button clicked")
        pid_field = self.set_pid_field.text()
        pid_field = pid_field.split(":")
        pid_type = pid_field[0]
        pid_params = pid_field[1]
        self.sock.sendall(f"set{pid_type}PID:{pid_params}".encode('utf-8'))
        print(f"Setting {pid_type}PID parameters: {pid_params}")

    def handle_connect_clicked(self):
        print("Connect clicked")
        self.connect_button.setText("Connecting")
        if self.connect_state == "disconnect":
            self.connect_state = "connect"
            self.tabs.addTab(self.tab2, "Graph")
            self.tabs.addTab(self.tab3, "Map")
            self.init_map()

            ssid = self.dropdown.currentText()
            print(f"[INFO] Attempting to connect to SSID: {ssid}")
            port = 3333

            # 1) Create TCP socket to the ESP32 at 192.168.4.1:3333
            try:
                self.sock = connect_to_esp32_tcp(ssid, port)
                self.connect_button.setText("Disconnect")
                self.startTime = time.time()
                self.reader_thread = threading.Thread(target=self.read_from_socket, daemon=True)
                self.reader_thread.start()  # Start reading from the socket
                self.socket_timer.start()
            except Exception as e:
                print(f"[ERROR] Could not connect socket: {e}")
                self.connect_button.setText("Connect")
                self.connect_state = "disconnect"
                self.socket_timer.stop()
                self.sock = None
        else:
            print("Disconnect clicked")
            self.connect_state = "disconnect"
            self.sock.sendall(f"disconnect".encode('utf-8'))
            self.connect_button.setText("Connect")
            # self.sock.close()
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

    def update_rec_data(self):
        with self.lock:
            if self.latest_data is None:
                return
            if self.start_state == "stop" and self.get_pid_state == False:
                return
            
            data_dict = {"description": "unknown"}
            chunk = self.latest_data
            line = chunk.split("\n")
            # line = line.strip()
            for l in line:
                if "description" in l and "end" in l:
                    data_dict = self.process_incoming_line(l[:-3])
                    break
            
            # Check the message part of the received data to determine the kind of message.   
            match data_dict["description"]:
                case "data":
                    self.update_table(data_dict)
                    self.update_graph(data_dict)
                    self.update_map_values(data_dict)
                case "parameters":
                    self.get_pid_label.setText(f"PID parameters: {data_dict['parameters']}")
                    self.get_pid_state = False
                case "stateUpdate":
                    self.update_state_label(data_dict["state"][:-3])
                case "PWMTest":
                    self.update_table(data_dict)
                    self.update_graph(data_dict)
                case "finishLineFound":
                    self.update_state_label("Finish line found, stopping robot")
                case _:
                    pass

    def update_map_values(self, data_dict):
        # Update the data dictionary
        map_scale_factor = 5
        x_pos = 0
        y_pos = 0
        velocity = 0

        for key, value in data_dict.items():
            if key == "X-position":
                x_pos = round(float(value), 3)*map_scale_factor
                self.data[key] = x_pos
            elif key == "Y-position":
                y_pos = round(float(value), 3)*map_scale_factor
                self.data[key] = y_pos
            elif key == "Velocity":
                velocity = round(float(value), 3)
                self.data[key] = velocity

        self.map_pts.append((x_pos, y_pos))
        self.draw_map(self.map_pts, (x_pos, y_pos), velocity)

    def read_from_socket(self):
        """
        Called periodically by QTimer to read available data from the socket (non-blocking).
        If there's a complete line, parse it, update table and graph.
        """

        while True:
            if self.connect_state == "connect" or self.get_pid_state:
                try:
                    data = self.sock.recv(1024)
                    if data:
                        with self.lock:
                            self.latest_data = data.decode('utf-8', errors='ignore')
                except socket.timeout:
                    # No data available, not an error
                    continue
                except BlockingIOError:
                    continue


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
        # Clear out existing rows
        self.table.clear()

        # Extract keys and values
        keys = list(data_dict.keys())
        values = list(data_dict.values())

        # Set up table
        self.table.setRowCount(len(keys) - 1)
        self.table.setHorizontalHeaderLabels(["Signal", "Value", "Average", "Min", "Max", "Number of samples"])
        

        for i, (k, v) in enumerate(data_dict.items()):
            if k != "description":
                self.dropdownGraphValue.setMaxCount(len(keys))
                self.dropdownGraphValue.addItem(k)
                if k not in self.current_tot:
                    self.current_tot[k] = 0
                    self.current_min[k] = 10000
                    self.current_max[k] = -10000
                self.current_tot[k] += float(v)
                self.current_min[k] = min(self.current_min[k], float(v))
                self.current_max[k] = max(self.current_max[k], float(v))
                self.table.setItem(i - 1, 0, QTableWidgetItem(k))
                self.table.setItem(i - 1, 1, QTableWidgetItem(v))
                self.table.setItem(i - 1, 2, QTableWidgetItem(str(round((self.current_tot[k])/self.counter, 2))))
                self.table.setItem(i - 1, 3, QTableWidgetItem(str(round(self.current_min[k], 2))))
                self.table.setItem(i - 1, 4, QTableWidgetItem(str(round(self.current_max[k], 2))))
                self.table.setItem(i - 1, 5, QTableWidgetItem(str(self.counter)))
        self.table.resizeColumnsToContents()

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
    dark_palette.setColor(QPalette.PlaceholderText, QColor(255, 255, 255))

    app.setPalette(dark_palette)

def main():
    # Make sure that file exists
    SRC_DIR = os.path.dirname(os.path.abspath(str(sys.modules['__main__'].__file__)))
    calibration_file_exists = os.path.exists(os.path.join(SRC_DIR , 'PWM_calibration.txt'))
    if calibration_file_exists:
        print("PWM_calibration.txt file exists")
    else:
        print("PWM_calibration.txt file does not exist. Creating a new file")
        CD = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(os.path.join(SRC_DIR , 'PWM_calibration.txt'), 'w') as f:
            f.write(f"Calibration file created: {CD}\n")

    app = QApplication(sys.argv)
    apply_dark_mode(app)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()