#include <MKRWAN.h>

byte byte_battery_voltage = 0;
byte byte_current_state = 0;
byte byte_alarm_state = 0;

String appEui = "0000000000000000";
String appKey = "00000000000000000000000000000000"; 

enum State {  /* Stat bit output: */
  FAULT,      /* 0 - 0000 */
  OPENING,    /* 1 - 0001 */
  OPENINGKO,  /* 2 - 0010 */
  OPEN,       /* 3 - 0011 */
  OPENKO,     /* 4 - 0100 */
  CLOSING,    /* 5 - 0101 */
  CLOSED,     /* 6 - 0110 */
  STOPPED,    /* 7 - 0111 */
  UNK         /* 8 - 1000 */
};

enum Receive {
  RX_OPEN,
  RX_OPENKO,
  RX_CLOSE,
  RX_STOP,
  RX_PING,
  RX_RESET,
  RX_LOCK_ON,
  RX_LOCK_OFF
};

bool debug = false;

int state = State::UNK;
int alarm = 0;
int moving = false;
bool lock = false;
bool flasher = false;
unsigned long open_warn_timer = 0;
bool class_c_enabled = false;

/* Timers */

unsigned long check_state_timer = 0;
unsigned long auto_send_payload_timer = 0;
unsigned long battery_read_timer = 0;
unsigned long flasher_timer = 0;

/* Pins */

const int stat_bit_0 = 0;
const int stat_bit_1 = 1;
const int stat_bit_2 = 2;
const int stat_bit_3 = 3;

const int pin_open = A5;
const int pin_open_keep_open = A6;

const int pin_battery_read = A0;
const int pin_reset_328 = A2;
const int pin_warn_led = A3;
const int pin_stat_led = A4;
const int pin_lora_led = A1;

const int pin_lock_out = 6;


LoRaModem modem;

void setup() {

  pinMode(stat_bit_0, INPUT);
  pinMode(stat_bit_1, INPUT);
  pinMode(stat_bit_2, INPUT);
  pinMode(stat_bit_3, INPUT);

  pinMode(pin_open, OUTPUT);
  pinMode(pin_open_keep_open, OUTPUT);

  pinMode(pin_battery_read, INPUT);
  pinMode(pin_reset_328, OUTPUT);
  pinMode(pin_warn_led, OUTPUT);
  pinMode(pin_stat_led, OUTPUT);
  pinMode(pin_lora_led, OUTPUT);

  pinMode(pin_lock_out, OUTPUT);

  digitalWrite(pin_open, LOW);
  digitalWrite(pin_open_keep_open, LOW);

  digitalWrite(pin_reset_328, HIGH);
  digitalWrite(pin_warn_led, LOW);
  digitalWrite(pin_stat_led, LOW);
  digitalWrite(pin_lora_led, LOW);

  if(debug) {
    byte_battery_voltage = 125;
  }

  Serial.begin(9600);

  if (!modem.begin(AU915)) {

    Serial.println("Failed to start module!");
    while (1) {}
  }

  Serial.print("Your module version is: ");

  Serial.println(modem.version());

  Serial.print("Your device EUI is: ");

  Serial.println(modem.deviceEUI());

  /* Set Regional Parameters and TTN specification: channels 8 to 15 plus 65 (AU915 - 928) */

  modem.sendMask("ff000000f000ffff00020000");
  
  Serial.println(modem.getChannelMask());
  
  join();

  getCurrentState();
  readBatteryVoltage();
}

bool updateLeds() {

  if(moving) {
    digitalWrite(pin_stat_led, flasher);
  } else {
    digitalWrite(pin_stat_led, LOW);
  }

  if(millis() > open_warn_timer && (state == State::OPEN || state == State::CLOSING)) {
    digitalWrite(pin_warn_led, HIGH);
  } else {
    digitalWrite(pin_warn_led, LOW);
  }

  if(!class_c_enabled) {
    digitalWrite(pin_lora_led, flasher);
  }
}

bool getCurrentState() {

  if(debug) {
    state = State::CLOSED;
    moving = false;
    return false;
  }

  moving = false;

  int old_state = state;

  if(!digitalRead(stat_bit_0) && !digitalRead(stat_bit_1) && !digitalRead(stat_bit_2) && !digitalRead(stat_bit_3)) {
    state = State::FAULT;
  }

  if(!digitalRead(stat_bit_0) && !digitalRead(stat_bit_1) && !digitalRead(stat_bit_2) && digitalRead(stat_bit_3)) {
    state = State::OPENING;
    moving = true;
  }

  if(!digitalRead(stat_bit_0) && !digitalRead(stat_bit_1) && digitalRead(stat_bit_2) && !digitalRead(stat_bit_3)) {
    state = State::OPENINGKO;
    moving = true;
  }

  if(!digitalRead(stat_bit_0) && !digitalRead(stat_bit_1) && digitalRead(stat_bit_2) && digitalRead(stat_bit_3)) {
    state = State::OPEN;
  }

  if(!digitalRead(stat_bit_0) && digitalRead(stat_bit_1) && !digitalRead(stat_bit_2) && !digitalRead(stat_bit_3)) {
    state = State::OPENKO;
  }

  if(!digitalRead(stat_bit_0) && digitalRead(stat_bit_1) && !digitalRead(stat_bit_2) && digitalRead(stat_bit_3)) {
    state = State::CLOSING;
    moving = true;
  }

  if(!digitalRead(stat_bit_0) && digitalRead(stat_bit_1) && digitalRead(stat_bit_2) && !digitalRead(stat_bit_3)) {
    state = State::CLOSED;
  }

  if(!digitalRead(stat_bit_0) && digitalRead(stat_bit_1) && digitalRead(stat_bit_2) && digitalRead(stat_bit_3)) {
    state = State::STOPPED;
  }

  if(digitalRead(stat_bit_0) && !digitalRead(stat_bit_1) && !digitalRead(stat_bit_2) && !digitalRead(stat_bit_3)) {
    state = State::UNK;
  }

  if(state == old_state) {
    return false;
  }

  if(state == State::OPEN) {
    open_warn_timer = millis() + 45000;
  }

  return true;
}

bool sendPayload() {

    if(lock) {
      byte_current_state = lowByte(state + 10);
    } else {
      byte_current_state = lowByte(state);
    }

    int err = 0;
  
    modem.beginPacket();

    modem.write(byte_battery_voltage);
    modem.write(byte_current_state);
    modem.write(byte_alarm_state);

    err = modem.endPacket(true);

    if (err > 0) {
        return true;
    }

    return false;
}


void join() {

  int connected = modem.joinOTAA(appEui, appKey);

  if (!connected) {

    Serial.println("Failed to connect. Retrying...");

    join();

  } else {
    
    Serial.println("Connected!");

    digitalWrite(pin_lora_led, HIGH);

    modem.setADR(true);

    if(modem.configureClass(CLASS_C)) {

      Serial.println("Modem set to Class C mode!");
      class_c_enabled = true;
    } else {
      Serial.println("Failed to set modem to Class C mode!");
    }
  }
}

void trigOpenClose(bool keep_open) {

  digitalWrite(pin_lock_out, LOW);

  delay(100);
  
  if(keep_open) {

    Serial.println("Triggering open keep open signal...");
    
    digitalWrite(pin_open_keep_open, HIGH);
    delay(1000);
    digitalWrite(pin_open_keep_open, LOW);
  } else {

    Serial.println("Triggering open signal...");
    
    digitalWrite(pin_open, HIGH);
    delay(1000);
    digitalWrite(pin_open, LOW);
  }

  if(lock) {

    digitalWrite(pin_lock_out, HIGH);
  }
}

void readBatteryVoltage() {

  byte_battery_voltage = 0;

  int readBatt = analogRead(pin_battery_read);
  float value = ((float)readBatt * 14) / 1023;

  Serial.println("Battery Voltage: ");
  Serial.println(value);

  if(value > 0 && value < 20) {
    byte_battery_voltage = value * 10;
  }
}

void loop() {

  /* Set LEDs: */

  updateLeds();

  if(millis() - flasher_timer > 500) {
    
    flasher = !flasher;
    flasher_timer = millis();
  }

  /* Set lock pin: */

  if(lock) {
    digitalWrite(pin_lock_out, HIGH);
  } else {
    digitalWrite(pin_lock_out, LOW);
  }

  /* Read battery voltage: */

  if(millis() - battery_read_timer > 60000) {
    
    readBatteryVoltage();
    battery_read_timer = millis();
  }

  /* Set alarm value: */

  alarm = 0;

  if(state == State::FAULT) {
    alarm = 2;
  }

  if(state == State::UNK) {
    alarm = 1;
  }

  byte_alarm_state = lowByte(alarm);

  /* Automatically send payload every 30 minutes: */

  if(millis() - auto_send_payload_timer > 1800000) {

    Serial.println("Payload Timer: Sending Payload...");
    
    if(sendPayload()) {
      
      Serial.println("Send success.");
    } else {
      Serial.println("Send fail.");
    }
    
    auto_send_payload_timer = millis();
  }

  /* Check for new state from 328P every second: */

  if(millis() - check_state_timer > 1000) {
    
    if(getCurrentState()) {

      /* State has changed: */

      delay(100);
      getCurrentState();

      sendPayload();
    }
    
    check_state_timer = millis();
  }

  /* Check for data coming in from modem: */

  char modem_data[32];
  int count = 0;

  while (modem.available()) {
    
    modem_data[count] = modem.read();
    ++count;
  }

  modem_data[count] = '\n';
  modem_data[count + 1] = '\0';

  modem.poll();

  if(modem_data[0] == Receive::RX_OPEN && count == 1) {
    
    Serial.println("Receive: OPEN");

    if(state == State::CLOSED || state == State::STOPPED || state == State::UNK) {
      trigOpenClose(false);
    }
  }

  if(modem_data[0] == Receive::RX_OPENKO && count == 1) {
    
    Serial.println("Receive: OPENKO");

    if(state == State::CLOSED || state == State::STOPPED || state == State::UNK) {
      trigOpenClose(true);
    }
  }

  if(modem_data[0] == Receive::RX_CLOSE && count == 1) {
    
    Serial.println("Receive: CLOSE");

    if(state == State::OPEN || state == State::OPENKO || state == State::STOPPED || state == State::UNK) {
      trigOpenClose(false);
    }
  }

  if(modem_data[0] == Receive::RX_STOP && count == 1) {

    if(moving) {
      trigOpenClose(false);
    }
  }

  if(modem_data[0] == Receive::RX_PING && count == 1) {
    
    Serial.println("Receive: PING");

    sendPayload();
  }

  if(modem_data[0] == Receive::RX_RESET && count == 1) {
    
    Serial.println("Receive: RESET");
    Serial.println("Sending reset signal...");

    digitalWrite(pin_reset_328, LOW);
    delay(1000);
    digitalWrite(pin_reset_328, HIGH);
    delay(3000);
  }

  if(modem_data[0] == Receive::RX_LOCK_ON && count == 1) {
    
    Serial.println("Receive: LOCK ON");

    lock = true;
    sendPayload();
  }

  if(modem_data[0] == Receive::RX_LOCK_OFF && count == 1) {
    
    Serial.println("Receive: LOCK OFF");

    lock = false;
    sendPayload();
  }
}
