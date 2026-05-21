#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>

/* Pin Definitions */
#define CE_PIN 9
#define CSN_PIN 10

#define LED1 2
#define LED2 3

#define SW1 4
#define SW2 5

#define jx1 A0
#define jy1 A1  
#define jx2 A2
#define jy2 A3  

#define jsw1 6
#define jsw2 7

#define JOYSTICK_up_threshold 660
#define JOYSTICK_down_threshold 550

/* Data Structures */
struct joystick
{
  int jx;
  int jy;
  int button;
};
enum State {
  HOME,
  NAVIGATION,
  COMMUNICATION,
  DEBUG,
  SETTINGS,
};


/* Global Objects */
LiquidCrystal_I2C lcd(0x27, 20, 4);
RF24 radio(CE_PIN, CSN_PIN);
String menu_items[] = {
  "Navigation",
  "Communication",
  "Debug",
  "Settings"
};
int8_t current_cursor_pos;
joystick right_joystick;
joystick left_joystick;
State current_state = HOME;

/* Function Prototypes */
int8_t initializeLCD();
void initializeRadio();
void homeScreen(int8_t current_cursor_pos);
void readJoystick(joystick* js, bool isLeft);
void updateCursorPosition(int8_t* current_cursor_pos, joystick* js);
bool isJoystickMoved(joystick* js);
bool isJoystickButtonPressed(joystick* js);
void writeToLcd(String message, int col, int row);
void setMenuPage(State state);
void debugPage();
void navigationPage();
void communicationPage();
void settingsPage();
void sendDataRadio(String data);
 

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);

  pinMode(jsw1, INPUT_PULLUP);
  pinMode(jsw2, INPUT_PULLUP);

  Serial.begin(115200);

  current_cursor_pos = initializeLCD();
  initializeRadio();
  homeScreen(current_cursor_pos);

  readJoystick(&right_joystick, false);
  readJoystick(&left_joystick, true);

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
}

void loop() {

  readJoystick(&right_joystick, false);
  readJoystick(&left_joystick, true);

  if (isJoystickMoved(&right_joystick) && current_state == HOME) {
    updateCursorPosition(&current_cursor_pos, &right_joystick);
    homeScreen(current_cursor_pos);
    delay(100);
  }
  if (isJoystickButtonPressed(&left_joystick) && current_state == HOME) {
    current_state = static_cast<State>(current_cursor_pos + 1);
    setMenuPage(current_state);
    delay(100);
  }

  if (isJoystickButtonPressed(&right_joystick) ){
    current_state = HOME;
    homeScreen(current_cursor_pos);
  }


  // if (((isJoystickMoved(&right_joystick)) || (isJoystickMoved(&left_joystick))) && (current_state == COMMUNICATION)) {
  //   communicationPage();
  // }

  if ((current_state == COMMUNICATION)) {
    communicationPage();
  }

  if ((current_state == NAVIGATION)) {
    readJoystick(&right_joystick, false);
    readJoystick(&left_joystick, true);
    sendDataRadio("RJX=" + String(right_joystick.jx) + "|RJY=" + String(right_joystick.jy)+"LJX=" + String(left_joystick.jx) + "|LJY=" + String(left_joystick.jy));
    delay(10);
  }
  
  if (current_state == DEBUG) {
    debugPage();
    delay(50);
  }

  //setMenuPage(current_state);

  //delay(10);
}

void writeToLcd(String message, int col, int row) {
  lcd.setCursor(col, row);
  lcd.print(message);
}

int8_t initializeLCD() {
  lcd.init();
  lcd.backlight();
  writeToLcd("Welcome to RC 1.0", 0, 0);
  writeToLcd("By ISMAIL HAMROUNI", 0, 1);
  delay(2000);
  lcd.clear();
  return 0;
}

void initializeRadio() {
  const byte address[6] = "00001";
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(address);
  radio.stopListening();
}

void sendMessage(String msg) {
  char buffer[32];                  // taille max payload NRF24 = 32 bytes
  msg.toCharArray(buffer, 32);      // conversion String -> char[]

  bool success = radio.write(&buffer, sizeof(buffer));

  if (success) {
    Serial.println("Message envoye !");
    digitalWrite(LED1, HIGH);
  } else {
    Serial.println("Echec envoi");
    digitalWrite(LED1, LOW);
  }
}

void sendDataRadio(String data) {
  radio.stopListening();
  sendMessage(data);
}

void homeScreen(int8_t current_cursor_pos) {
  lcd.clear();

  lcd.setCursor(0, (int)current_cursor_pos);
  lcd.print("> ");

  for (int i = 0; i < 4; i++) {
    lcd.setCursor(2, i);
    lcd.print(menu_items[i]);
  }

}

void readJoystick(joystick* js, bool isLeft) {  
  if (isLeft) {
    js->jx = analogRead(jx2);
    js->jy = analogRead(jy2);
    js->button = digitalRead(jsw2);
  } else {
    js->jx = analogRead(jx1);
    js->jy = analogRead(jy1);
    js->button = digitalRead(jsw1);
  }
}

void updateCursorPosition(int8_t* current_cursor_pos, joystick* js) {
  if (js->jx > JOYSTICK_up_threshold) {
    (*current_cursor_pos)--;
    if (*current_cursor_pos < 0) {
      *current_cursor_pos = 3;
    }
  } else if (js->jx < JOYSTICK_down_threshold) {
    (*current_cursor_pos)++;
    if (*current_cursor_pos > 3) {
      *current_cursor_pos = 0;
    }
  }
}

bool isJoystickMoved(joystick* js) {
  readJoystick(js, js == &left_joystick);
  if (js->jx > JOYSTICK_up_threshold || js->jx < JOYSTICK_down_threshold) {
    return true;
  }
  else return false;
}

bool isJoystickButtonPressed(joystick* js) {
  readJoystick(js, js == &left_joystick);
  return js->button == LOW;
} 


void debugPage() {
  lcd.clear();
  writeToLcd("=======Debug=======", 0, 0);
  writeToLcd("RJX: " + String(right_joystick.jx) + " | RJY: " + String(right_joystick.jy), 0, 1);
  writeToLcd("LJX: " + String(left_joystick.jx) + " | LJY: " + String(left_joystick.jy), 0, 2);
  writeToLcd("SW1: " + String(digitalRead(SW1)) + " | SW2: " + String(digitalRead(SW2)), 0, 3);
}

void navigationPage() {
  lcd.clear();
  writeToLcd("===Navigation===", 0, 0);
  // Add navigation-specific info here
  writeToLcd("  ^              ^  ", 0, 1);
  writeToLcd("< O >          < O >", 0, 2);
  writeToLcd("  v              v  ", 0, 3);
  readJoystick(&right_joystick, false);
  readJoystick(&left_joystick, true);
  sendDataRadio("RJX=" + String(right_joystick.jx) + "|RJY=" + String(right_joystick.jy)+"|LJX=" + String(left_joystick.jx) + "|LJY=" + String(left_joystick.jy));

}

void communicationPage() {
  lcd.clear();
  writeToLcd("===Communication===", 0, 0);
  // Add communication-specific info here
  readJoystick(&right_joystick, false);
  readJoystick(&left_joystick, true);
  sendDataRadio("RJX=" + String(right_joystick.jx) + "|RJY=" + String(right_joystick.jy)+"LJX=" + String(left_joystick.jx) + "|LJY=" + String(left_joystick.jy));
  writeToLcd("RJX: " + String(right_joystick.jx) + " | RJY: " + String(right_joystick.jy), 0, 1);
  writeToLcd("LJX: " + String(left_joystick.jx) + " | LJY: " + String(left_joystick.jy), 0, 2);
  delay(50);
}

void settingsPage() {
  lcd.clear();
  writeToLcd("===Settings===", 0, 0);
  writeToLcd("Calibration ?", 0, 1);
}

void setMenuPage(State state) {
  lcd.clear();
  switch (state) {
    case NAVIGATION:
      navigationPage();
      break;
    case COMMUNICATION:
      communicationPage();
      break;
    case DEBUG:
      debugPage();
      break;
    case SETTINGS:
      settingsPage();
      break;
    default:
      homeScreen(current_cursor_pos);
  }
}