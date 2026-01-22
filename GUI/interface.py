from PyQt5 import uic
from PyQt5.QtWidgets import QApplication
import random
import threading
import time
import serial
import serial.tools.list_ports


class SerialThread(threading.Thread):
    def __init__(self, port="COM3", baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None
        self.pwm_slider_values = [0,0,0]

    def set_pwm_value(self,value,id_pwm):
        self.pwm_slider_values[id_pwm-1] = value

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            print(f"Connecté à {self.port}")
        except serial.SerialException:
            print(f"Impossible de se connecter à {self.port}")
            return
        id_pwm = 0
        while self.running:
            try:
                # 🔼 Envoi de la commande PWM1 à 25%
                for id_pwm in range(3):
                    cmd = f"PWM;{id_pwm+1};{self.pwm_slider_values[id_pwm]}\n"
                    self.ser.write(cmd.encode("utf-8"))
                    #print("Envoyé :", cmd.strip())
                    # 🔽 Attente de la réponse STM32
                    response = self.ser.readline()  # bloque jusqu'au timeout
                    if response:
                        try:
                            # print("Reçu :", response.decode("utf-8").strip())
                            current_pwm_value = response.decode("utf-8").strip().split(":")[1]
                        except UnicodeDecodeError:
                            # print("Reçu (brut) :", response)
                            current_pwm_value = response.split(":")[1]
                    else:
                        print("[PWM] ⚠️ Timeout : aucune réponse du STM32")
                
                cmd = f"ADC;0;0\n"
                self.ser.write(cmd.encode("utf-8"))
                # print("Envoyé :", cmd.strip())
                # 🔽 Attente de la réponse STM32
                response = self.ser.readline()  # bloque jusqu'au timeout
                if response:
                    try:
                        #print("Reçu :", response.decode("utf-8").strip())
                        values = response.decode("utf-8").strip().split(":")[1].split(",")
                        adc_0 = values[0]
                        adc_1 = values[1]
                        # print(f"adc 0 : {adc_0} adc 1 : {adc_1}")
                        update_current_measurement(float(adc_0),float(adc_1),0.0)
                    except UnicodeDecodeError:
                        #print("Reçu (brut) :", response)
                        values = response.split(":")[1].split(",")
                else:
                    print("⚠️ Timeout : aucune réponse du STM32")

            except serial.SerialException:
                print("Erreur série")
                break

            # time.sleep(1)  # envoi toutes les 1 seconde

        self.close_serial()

    def close_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"Déconnecté de {self.port}")

    def stop(self):
        self.running = False
        self.join()

def random_float(x, y):
    return random.uniform(x, y)

def random_int(x, y):
    return random.randint(x, y)


def toggle_led(btn):
    if btn.property("ledState") == "off":
        btn.setProperty("ledState", "on")
        btn.setText("OFF")  # le texte change
    else:
        btn.setProperty("ledState", "off")
        btn.setText("ON")
    btn.style().unpolish(btn)
    btn.style().polish(btn)

def init_led_buttons(btn):
    btn.setProperty("ledState", "off")
    btn.setText("ON")

def update_label(slider,pwm_label,id_pwm):
    slider_value = slider.value()
    pwm_label.setText(f"{slider_value}%")
    thread_stm_serial.set_pwm_value(slider_value,id_pwm)


def update_current_measurement(ia,ib,ic):
    lcdNumber_ia.display(f"{ia:.1f}")
    lcdNumber_ib.display(f"{ib:.1f}")
    lcdNumber_ic.display(f"{ic:.1f}")

def update_voltage(volt):
    lcdNumber_voltage.display(f"{volt:.2f}")

def update_velocity(omega):
    lcdNumber_velocity.display(f"{omega:.0f}")


def on_close():
    print("Thread will stop...")
    stop_event.set()
    thread.join(2)
    thread_stm_serial.join(2)

def background_task(stop_event):
    while not stop_event.is_set():
        ia = random_float(0.0,20.0)
        ib = random_float(0.0,20.0)
        ic = random_float(0.0,20.0)

        velocity = random_float(0,500)
        voltage = random_float(0,40)

        # update_current_measurement(ia,ib,ic)
        # update_voltage(voltage)
        # update_velocity(velocity)
        time.sleep(0.5)



ports = serial.tools.list_ports.comports()
devices = []
for p in ports:
    print(f"""
Port      : {p.device}
Description: {p.description}
HWID      : {p.hwid}
VID       : {p.vid}
PID       : {p.pid}
Fabricant : {p.manufacturer}
""")
    devices.append(p.device)

app = QApplication([])
window = uic.loadUi("UI/main.ui")
stop_event = threading.Event()

ledOnButton_1 = window.ledOnButton_1
ledOnButton_1.clicked.connect(lambda: toggle_led(ledOnButton_1))
init_led_buttons(ledOnButton_1)

ledOnButton_2 = window.ledOnButton_2
ledOnButton_2.clicked.connect(lambda: toggle_led(ledOnButton_2))
init_led_buttons(ledOnButton_2)

ledOnButton_3 = window.ledOnButton_3
ledOnButton_3.clicked.connect(lambda: toggle_led(ledOnButton_3))
init_led_buttons(ledOnButton_3)

verticalSlider_pwm1 = window.verticalSlider_pwm1
verticalSlider_pwm2 = window.verticalSlider_pwm2
verticalSlider_pwm3 = window.verticalSlider_pwm3

pwm_value1 = window.pwm_value1
pwm_value2 = window.pwm_value2
pwm_value3 = window.pwm_value3

verticalSlider_pwm1.valueChanged.connect(lambda: update_label(verticalSlider_pwm1,pwm_value1,1))
verticalSlider_pwm2.valueChanged.connect(lambda: update_label(verticalSlider_pwm2,pwm_value2,2))
verticalSlider_pwm3.valueChanged.connect(lambda: update_label(verticalSlider_pwm3,pwm_value3,3))


lcdNumber_ia = window.lcdNumber_ia
lcdNumber_ib = window.lcdNumber_ib
lcdNumber_ic = window.lcdNumber_ic

lcdNumber_voltage = window.lcdNumber_voltage
lcdNumber_velocity = window.lcdNumber_velocity

serial_port_liste = window.serial_port_liste

serial_port_liste.addItems(["Choose serial port ... "])
serial_port_liste.setCurrentText("Choose serial port ... ")
# devices = ["/dev/ttyUSB0","/dev/ttyUSB1"]
for device in devices : 
    serial_port_liste.addItems([device])

update_current_measurement(12.1,1.0,0.0)
update_voltage(24.0)
update_velocity(0)

thread = threading.Thread(target=background_task, args=(stop_event,), daemon=True)
thread.start()

thread_stm_serial = SerialThread(port="/dev/ttyUSB0", baudrate=115200)
thread_stm_serial.start()

window.show()
app.aboutToQuit.connect(on_close)
app.exec_()
