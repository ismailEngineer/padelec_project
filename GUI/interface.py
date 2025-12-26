from PyQt5 import uic
from PyQt5.QtWidgets import QApplication
import random
import threading
import time
import serial


class SerialThread(threading.Thread):
    def __init__(self, port="COM3", baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connecté à {self.port}")
        except serial.SerialException:
            print(f"Impossible de se connecter à {self.port}")
            return

        while self.running:
            try:
                # Envoie du caractère 'A'
                self.ser.write(b"A")
                print("Envoyé : A")
            except serial.SerialException:
                print("Erreur d'écriture")
                break

            time.sleep(1)  # envoi toutes les 1 seconde

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

def update_label(slider,pwm_label):
    slider_value = slider.value()
    pwm_label.setText(f"{slider_value}%")


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

        update_current_measurement(ia,ib,ic)
        update_voltage(voltage)
        update_velocity(velocity)
        time.sleep(0.5)


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

verticalSlider_pwm1.valueChanged.connect(lambda: update_label(verticalSlider_pwm1,pwm_value1))
verticalSlider_pwm2.valueChanged.connect(lambda: update_label(verticalSlider_pwm2,pwm_value2))
verticalSlider_pwm3.valueChanged.connect(lambda: update_label(verticalSlider_pwm3,pwm_value3))


lcdNumber_ia = window.lcdNumber_ia
lcdNumber_ib = window.lcdNumber_ib
lcdNumber_ic = window.lcdNumber_ic

lcdNumber_voltage = window.lcdNumber_voltage
lcdNumber_velocity = window.lcdNumber_velocity

update_current_measurement(12.1,1.0,0.0)
update_voltage(32.22)
update_velocity(350)

thread = threading.Thread(target=background_task, args=(stop_event,), daemon=True)
thread.start()

thread_stm_serial = SerialThread(port="COM3", baudrate=115200)
thread_stm_serial.start()

window.show()
app.aboutToQuit.connect(on_close)
app.exec_()
