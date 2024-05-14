// MOR_IV 17.10.2018

#include <Arduino.h>
#include <EEPROM.h>
#include <LCD_1602_RUS.h>
#include <Packet.h>
#include <string.h>

LCD_1602_RUS lcd(0x27, 20, 4);

#define FLOW_PIN 3
// KEYBOARD 4,5,6,7,8
const uint8_t keyboard_pins[] = {7, 4, 6, 5, 8};
#define MOSFET_PIN 9
#define SELECTION_VALVE_PIN 10
#define TENG_ONE_PIN 11
#define COOLER_PIN 12
#define TENG_TWO_PIN 13
#define BUZZER_PIN A2

// A4 SDA - display
// A5 SCL - display
#define HEAD_FULL_PIN A6

const float temp_err = 999;

#define COOLER_START_TEMP 70
#define PWM_MAX 1023
#define PUMP_CONTROL_SECOND 10
#define CALCULATE_TIME 8000
#define SELECTION_VALVE_COEFF                                                  \
  2100 //литров в час при полном открытии клапана отбора
#define SELECTION_VALVE_TIME 5000 //время цикла клапана мс
#define SELECTION_VALVE_OPEN_TIME 60 //время на открытие клапана мс

#define RECT_DELTA_PAUSE 180000
#define SERIAL_TIME 200 //отправка сообщений на SERIAL
#define SERVO_HEAD 30
#define SERVO_BODY 53
#define SERVO_TAIL 85
#define END_DATA_SAVE_TIME 1200000

// ID пакетов------------------
#define PACKET_TEMP_CUBE_ID 251
#define PACKET_TEMP_OUT_ID 252
#define PACKET_TEMP_TSA_ID 253
#define PACKET_SERVO_ID 254
#define PACKET_CONTROL_SERVO_ID 255
//----------------------------
#define END_DATA_EEPROM_ADDR 400

String STRING_ERR = "err";
String STRING_RECT_1 = "РЕКТ1";
String STRING_RECT_2 = "РЕКТ2";
String STRING_OFF = "Выкл";
String STRING_END = "Конец";
String STRING_OVERCLOCK = "Разгон";
String STRING_STABILIZATION = "Стаб";
String STRING_HEAD = "Голова";
String STRING_BODY = "Тело";
String STRING_TAIL = "Хвост";
String STRING_PROCESS = "Процесс";
String STRING_MANUAL = "Ручной";
String STRING_ERROR_TSA = "ERR_ТСА";
String STRING_ERROR_BARD = "ERR_БАРДА";
String STRING_ERROR_OUTPUT = "ERR_ВЕРХ";
String STRING_ERROR_SERVO = "ERR_ДРУГ";
String STRING_NBK_MODE = "НБК";
String STRING_CURSOR = ">";

unsigned long tick[PUMP_CONTROL_SECOND];

enum BuzzerType {
  BUZZER_INFO,
  BUZZER_ERROR,
  BUZZER_END,
  BUZZER_BUTTON,
  BUZZER_NONE
};
class Buzzer {
public:
private:
  bool enabled = false;
  BuzzerType select_type = BUZZER_NONE;
  unsigned long next = 0;
  byte b = 0;

  void to(BuzzerType t) {
    switch (t) {
    case BUZZER_INFO:
      tone(BUZZER_PIN, 2000, 2000);
      break;
    case BUZZER_ERROR:
      if (next < millis()) {
        tone(BUZZER_PIN, 2200, 300);
        if (b < 3) {
          next = millis() + 400;
          b++;
        } else {
          next = millis() + 1000;
          b = 0;
        }
      }
      break;
    case BUZZER_END:
      if (next < millis()) {
        tone(BUZZER_PIN, 2000, 1000);
        next = millis() + 2000;
      }
      break;
    case BUZZER_BUTTON:
      tone(BUZZER_PIN, 4000, 100);
      break;
    default:
      break;
    }
  }

public:
  void sing(BuzzerType t = BUZZER_NONE) {
    if (t != BUZZER_NONE) {
      to(t);
    } else {
      to(select_type);
    }
  }

  bool isEnabled() { return enabled; }

  void setEnabled(bool enabled) { this->enabled = enabled; }

  void setBuzzerType(BuzzerType select_type) {
    this->select_type = select_type;
  }
};
Buzzer buzzer;

uint8_t dataVersion = 87;
struct Data {
  uint8_t version;
  float pump_speed;
  float pump_coeff;
  float tsa;
  float nbk_bard;
  float nbk_output;
  float nbk_delta;
  uint16_t nbk_watt;
  uint8_t nbk_to_myself;
};

struct RectData {
  float tsa;
  float rect_cube_tail;
  float rect_cube_end;
  float rect_output;
  float rect_delta;
  float rect_delta_tail;
  uint16_t rect_watt;
  uint16_t rect_speed_head;
  uint16_t rect_speed_body;
  uint8_t rect_speed_reduction;
  uint8_t rect_to_myself;
};
struct EndData {
  float end_cube_temp;
  uint8_t end_status;
};
EndData end_data = {};
void endDataUpdate();
Data data = {};
RectData data_1 = {};
RectData data_2 = {};
class RectMode {
private:
  RectData *data;
  String name;

public:
  RectMode(RectData *data, String name) {
    this->data = data;
    this->name = name;
  }

  String getName() { return name; }

  RectData *getData() { return data; }
};
RectMode rect_1(&data_1, STRING_RECT_1);
RectMode rect_2(&data_2, STRING_RECT_2);
RectMode *activeMode = &rect_1;

class EEPROMHandler {
private:
  unsigned long next_save = 0;
  unsigned long next_save_end_data = 0;
  void save() {
    int addr = 0;
    EEPROM.put(addr, data);
    addr += sizeof(data);
    EEPROM.put(addr, data_1);
    addr += sizeof(data_1);
    EEPROM.put(addr, data_2);
  }
  void initData() {
    data.version = dataVersion;
    data.pump_speed = 15.5;
    data.pump_coeff = 1.95;
    data.tsa = 42;
    data.nbk_bard = 98.7;
    data.nbk_output = 90.2;
    data.nbk_delta = 0.3;
    data.nbk_watt = 3000;
    data.nbk_to_myself = 5;

    data_1.rect_cube_tail = 91;
    data_1.rect_cube_end = 96;
    data_1.rect_output = 45;
    data_1.rect_delta = 0.3;
    data_1.rect_delta_tail = 0.5;
    data_1.rect_watt = 2800;
    data_1.rect_speed_head = 150;
    data_1.rect_speed_body = 2100;
    data_1.rect_speed_reduction = 10;
    data_1.rect_to_myself = 60;

    data_2.rect_cube_tail = 80;
    data_2.rect_cube_end = 86;
    data_2.rect_output = 40;
    data_2.rect_delta = 0.2;
    data_2.rect_delta_tail = 0.5;
    data_2.rect_watt = 2800;
    data_2.rect_speed_head = 150;
    data_2.rect_speed_body = 2100;
    data_2.rect_speed_reduction = 10;
    data_2.rect_to_myself = 60;

    end_data.end_cube_temp = 0;
    end_data.end_status = 0;
  }
  void saveEndData() { EEPROM.put(END_DATA_EEPROM_ADDR, end_data); }

public:
  void load() {
    int addr = 0;
    EEPROM.get(addr, data);
    addr += sizeof(data);
    EEPROM.get(addr, data_1);
    addr += sizeof(data_1);
    EEPROM.get(addr, data_2);
    addr += sizeof(data_2);
    EEPROM.get(END_DATA_EEPROM_ADDR, end_data);
    if (data.version != dataVersion) {
      initData();
      saveTask();
      saveEndDataTask();
    }
  }
  void saveTask() { next_save = millis() + 10000; }
  void check() {
    if (next_save < millis() && next_save > 0) {
      save();
      next_save = 0;
    }
    if (next_save_end_data < millis() && next_save_end_data > 0) {
      saveEndData();
      next_save_end_data = 0;
    }
  }

  void saveEndDataTask() { next_save_end_data = millis() + 10000; }
};
EEPROMHandler eepromHandler;

class Pump {
private:
  float all_pulses = 0;
  float pulses = 0;
  unsigned long nextWriteTime = 0;
  unsigned long nextCalculateTime = 0;
  bool enabled;
  bool sleep = true;
  bool full = false;
  float accuracy = 0.35F;

public:
  uint16_t p = 512;
  bool manual = false;

  bool calibration = false;

  bool isEnabled() { return enabled; }
  void setSleep(bool s) {
    sleep = s;
    if (s == true) {
      pwm(0);
    }
  }
  bool isSleep() { return sleep; }
  void pwm(uint16_t l = 9999) {
    if (sleep && !calibration) {
      l = 0;
    }
    if (l != 9999) {
      p = l;
    }
    uint16_t a = PWM_MAX - p;
    if (a == 255) {
      a = 256;
    }
    enabled = p > 0 ? true : false;
    if (a >= PWM_MAX) {
      digitalWrite(MOSFET_PIN, HIGH);
      return;
    }
    if (a <= 0) {
      digitalWrite(MOSFET_PIN, LOW);
      return;
    }
    analogWrite(MOSFET_PIN, a);
  }

  void calibrate() {
    if (!calibration) {
      calibration = true;
      pwm(450);
      pulses = 0;
    } else {
      pwm(0);
      if (pulses == 0) {
        pulses = 200;
      }
      data.pump_coeff = pulses / 200;
      calibration = false;
      pulses = 0;
    }
  }
  void pulse() {
    all_pulses++;
    pulses++;
  }

  float getSpeed() {
    float s = 0;
    for (uint8_t i = 0; i < PUMP_CONTROL_SECOND; i++) {
      s = s + tick[i];
    }
    s = s / PUMP_CONTROL_SECOND;
    if (s == 0) {
      return 0;
    }
    float lh = s / data.pump_coeff * 3.6F;
    return lh;
  }

  void writePulses() {
    if (nextWriteTime > millis()) {
      return;
    }
    nextWriteTime = millis() + 1000;
    if (calibration) {
      return;
    }
    for (uint8_t i = PUMP_CONTROL_SECOND - 1; i > 0; i--) {
      tick[i] = tick[i - 1];
    }
    tick[0] = pulses;
    pulses = 0;
  }

  void calculate() {
    if (sleep) {
      return;
    }
    if (nextCalculateTime > millis()) {
      return;
    }
    nextCalculateTime = millis() + CALCULATE_TIME;
    if (manual || calibration) {
      return;
    }
    if (getSpeed() < 0.5F) {
      p = PWM_MAX;
      pwm();
      full = true;
      return;
    }
    if (full) {
      p = 450;
      pwm();
      full = false;
      return;
    }
    float c = getSpeed() - data.pump_speed;
    if (c == 0) {
      return;
    }
    if (c < accuracy / 2 && c > -accuracy / 2) {
      return;
    } else if (c < accuracy && c > -accuracy) {
      c > 0 ? p-- : p++;
    } else if (c < accuracy * 3 && c > -accuracy * 3) {
      c > 0 ? p -= 2 : p += 2;
    } else if (c < accuracy * 7 && c > -accuracy * 7) {
      c > 0 ? p -= 3 : p += 3;
    } else {
      float c = getSpeed() / data.pump_speed;
      if (c > 1.04F) {
        c = 1.04F;
      } else if (c < 0.96F) {
        c = 0.96F;
      }
      p = p / c;
    }
    if (p > PWM_MAX) {
      p = PWM_MAX;
    }
    pwm();
  }

  float getLiters() {
    float l = all_pulses / data.pump_coeff / 1000;
    l *= 10;
    l = floor(l + 0.5);
    l /= 10;
    return l;
  }
};
Pump pump;
class Temperature {
private:
  float temp[3] = {temp_err, temp_err, temp_err};

public:
  void setTsaTemp(float f) { temp[0] = f; }
  void setCubeTemp(float f) { temp[1] = f; }
  void setOutTempTemp(float f) { temp[2] = f; }
  float getTsaTemp() { return temp[0]; }
  float getBardTemp() { return temp[1]; }
  float getCubeTemp() { return getBardTemp(); }
  float getOutputTemp() { return temp[2]; }
};
Temperature temperature;
class PacketHandler {
private:
  unsigned long next_send = 0;
  unsigned long servo_last = 0;
  uint16_t servo = SERVO_HEAD;
  Packet packet;

public:
  void send() {
    if (next_send > millis()) {
      return;
    }
    next_send = millis() + SERIAL_TIME;
    packet.init(PACKET_SERVO_ID, servo);
    packet.send();
    packet.clear();
  }

  void take() {
    if (packet.avaible()) {
      if (packet.isValid()) {
        float f = 0;
        switch (packet.getId()) {
        case PACKET_TEMP_CUBE_ID:
          f = static_cast<float>(packet.getVal());
          f = f / 100;
          temperature.setCubeTemp(f);
          break;
        case PACKET_TEMP_OUT_ID:
          f = static_cast<float>(packet.getVal());
          f = f / 100;
          temperature.setOutTempTemp(f);
          break;
        case PACKET_TEMP_TSA_ID:
          f = static_cast<float>(packet.getVal());
          f = f / 100;
          temperature.setTsaTemp(f);
          break;
        case PACKET_CONTROL_SERVO_ID:
          if (packet.getVal() == servo) {
            servo_last = millis();
          }
          break;
        default:
          break;
        }
      }
    }
  }

  void setServo(uint16_t i) { servo = i; }
  unsigned long getServoLastAnswerTime() { return servo_last; }
};
PacketHandler packetHandler;
class Relay {
private:
  bool teng_one = false;
  bool teng_two = false;
  bool selection_valve = false;
  bool cooler = false;
  unsigned long selection_valve_open_time = 0;

public:
  void enableOne() {
    digitalWrite(TENG_ONE_PIN, LOW);
    teng_one = true;
  }
  void disableOne() {
    digitalWrite(TENG_ONE_PIN, HIGH);
    teng_one = false;
  }
  bool isEnabledOne() { return teng_one; }
  void enableTwo() {
    digitalWrite(TENG_TWO_PIN, LOW);
    teng_two = true;
  }
  void disableTwo() {
    digitalWrite(TENG_TWO_PIN, HIGH);
    teng_two = false;
  }
  bool isEnabledTwo() { return teng_two; }

  void openSelectionValve() {
    digitalWrite(SELECTION_VALVE_PIN, HIGH);
    selection_valve = true;
    selection_valve_open_time = millis();
  }
  void closeSelectionValve() {
    digitalWrite(SELECTION_VALVE_PIN, LOW);
    selection_valve = false;
  }
  bool isOpenSelectionValve() { return selection_valve; }
  unsigned long getSelectionValveOpenTime() {
    return selection_valve_open_time;
  }
  void enableCooler() {
    digitalWrite(COOLER_PIN, LOW);
    cooler = true;
  }
  void disableCooler() {
    digitalWrite(COOLER_PIN, HIGH);
    cooler = false;
  }
  bool isEnabledCooler() { return cooler; }
  void setup() {
    disableOne();
    disableTwo();
    closeSelectionValve();
    disableCooler();
  }
};
Relay relay;
enum Status {
  OFF,
  OVERCLOCK,
  STABILIZATION,
  HEAD,
  BODY,
  PROCESS,
  TAIL,
  END,
  MANUAL,
  ERROR_TSA,
  ERROR_BARD,
  ERROR_OUTPUT,
  ERROR_SERVO
};
enum Mode { NBK_MODE, RECT_MODE };
class NBK {
private:
  unsigned long next_run = 0;
  uint8_t error_braga = 0;
  uint8_t error_tsa = 0;
  uint8_t error_bard = 0;
  uint8_t error_output = 0;
  uint8_t overclock_delay = 0;
  uint8_t head_full_delay = 0;
  uint8_t rect_pause_delay = 0;
  uint8_t rect_cancel_pause_delay = 0;
  uint8_t tail_delay = 0;
  uint8_t end_delay = 0;
  Status status = OFF;
  unsigned long stab_end = 0;
  Mode mode = NBK_MODE;
  unsigned long start_time = 0;
  unsigned long stop_time = 0;
  uint16_t real_speed_body = 0;
  float start_body_temp = 0;
  uint16_t selection_valve_open_time = 0;
  bool pause_body = false;
  bool pause_tail = false;
  unsigned long pause_start_time = 0;
  void overclock() {
    if (!relay.isEnabledOne()) {
      relay.enableOne();
    }
    if (!relay.isEnabledTwo()) {
      relay.enableTwo();
    }
    if (!pump.isSleep()) {
      pump.setSleep(true);
    }
  }
  void process() {
    if (!relay.isEnabledOne()) {
      relay.enableOne();
    }
    if (relay.isEnabledTwo()) {
      relay.disableTwo();
    }
    if (!pump.isSleep()) {
      pump.setSleep(true);
    }
  }
  void stop() {
    if (relay.isEnabledOne()) {
      relay.disableOne();
    }
    if (relay.isEnabledTwo()) {
      relay.disableTwo();
    }
    if (!pump.isSleep()) {
      pump.setSleep(true);
    }
    if (relay.isOpenSelectionValve()) {
      relay.closeSelectionValve();
    }
  }
  void relayCheck() {
    switch (status) {
    case OFF:
      stop();
      break;
    case ERROR_TSA:
      stop();
      break;
    case ERROR_BARD:
      stop();
      break;
    case ERROR_SERVO:
      stop();
      break;
    case END:
      stop();
      break;
    case OVERCLOCK:
      overclock();
      break;
    case STABILIZATION:
      process();
      break;
    case HEAD:
      process();
      break;
    case BODY:
      process();
      break;
    case TAIL:
      process();
      break;

    case PROCESS:
      process();
      break;
    case MANUAL:
      if (pump.isSleep()) {
        pump.setSleep(false);
      }
      break;
    default:
      break;
    }
    if (!relay.isEnabledCooler()) {
      if (temperature.getOutputTemp() > COOLER_START_TEMP ||
          temperature.getCubeTemp() > COOLER_START_TEMP) {
        relay.enableCooler();
      }
    } else {
      if (temperature.getOutputTemp() < COOLER_START_TEMP ||
          temperature.getCubeTemp() < COOLER_START_TEMP) {
        relay.disableCooler();
      }
    }
  }
  void time(Status s) {
    unsigned long l = 0;
    switch (s) {
    case OFF:
      stop_time = millis();
      break;
    case END:
      stop_time = millis();
      break;
    case OVERCLOCK:
      start_time = millis();
      stab_end = 0;
      break;
    case STABILIZATION:
      if (mode == NBK_MODE) {
        l = data.nbk_to_myself;
      } else if (mode == RECT_MODE) {
        l = activeMode->getData()->rect_to_myself;
      }
      l = l * 60000;
      stab_end = l + millis();
      break;
    case HEAD:
      start_time = millis();
      stab_end = 0;
      packetHandler.setServo(SERVO_HEAD);
      break;
    case BODY:
      selection_valve_open_time = 0;
      packetHandler.setServo(SERVO_BODY);
      break;
    case PROCESS:
      stab_end = 0;
      break;
    case TAIL:
      pause_tail = false;
      selection_valve_open_time = 0;
      packetHandler.setServo(SERVO_TAIL);
      break;
    case MANUAL:
      start_time = millis();
      break;
    case ERROR_TSA:
      stop_time = millis();
      break;
    case ERROR_BARD:
      stop_time = millis();
      break;
    default:
      break;
    }
  }

public:
  unsigned long getStartTime() { return start_time; }
  unsigned long getStopTime() { return stop_time; }
  void setStatus(Status s) {
    status = s;
    time(s);
    endDataUpdate();
  }
  String getStringStatus(int8_t i = -1) {
    Status s;
    if (i == -1) {
      s = status;
    } else {
      s = static_cast<Status>(i);
    }
    switch (s) {
    case OFF:
      return STRING_OFF;
    case END:
      return STRING_END;
    case OVERCLOCK:
      return STRING_OVERCLOCK;
    case STABILIZATION:
      return STRING_STABILIZATION;
    case HEAD:
      return STRING_HEAD;
    case BODY:
      return STRING_BODY;
    case TAIL:
      return STRING_TAIL;
    case PROCESS:
      return STRING_PROCESS;
    case MANUAL:
      return STRING_MANUAL;
    case ERROR_TSA:
      return STRING_ERROR_TSA;
    case ERROR_BARD:
      return STRING_ERROR_BARD;
    case ERROR_OUTPUT:
      return STRING_ERROR_OUTPUT;
    case ERROR_SERVO:
      return STRING_ERROR_SERVO;
    default:
      break;
    }
    return STRING_ERR;
  }

  Status getStatus() { return status; }

  void nextStatus() {
    switch (status) {
    case OFF:
      setStatus(OVERCLOCK);
      break;
    case END:
      setStatus(OVERCLOCK);
      break;
    case OVERCLOCK:
      setStatus(STABILIZATION);
      break;
    case STABILIZATION:
      if (mode == NBK_MODE) {
        setStatus(PROCESS);
      } else if (mode == RECT_MODE) {
        setStatus(HEAD);
      }
      break;
    case HEAD:
      setStatus(BODY);
      break;
    case BODY:
      setStatus(TAIL);
      break;
    case TAIL:
      setStatus(OFF);
      real_speed_body = 0;
      selection_valve_open_time = 0;
      break;
    case PROCESS:
      setStatus(MANUAL);
      break;
    case MANUAL:
      setStatus(OFF);
      break;
    default:
      setStatus(OFF);
      break;
    }
  }
  void backStatus() {
    switch (status) {
    case OFF:
      if (getMode() == NBK_MODE) {
        setStatus(MANUAL);
      } else if (getMode() == RECT_MODE) {
        setStatus(TAIL);
      }
      break;
    case END:
      setStatus(PROCESS);
      break;
    case OVERCLOCK:
      setStatus(OFF);
      break;
    case STABILIZATION:
      setStatus(OVERCLOCK);
      break;
    case HEAD:
      setStatus(STABILIZATION);
      selection_valve_open_time = 0;
      break;
    case BODY:
      setStatus(HEAD);
      real_speed_body = 0;
      selection_valve_open_time = 0;
      break;
    case TAIL:
      setStatus(BODY);
      break;
    case PROCESS:
      setStatus(STABILIZATION);
      break;
    case MANUAL:
      setStatus(PROCESS);
      break;
    default:
      setStatus(OFF);
      break;
    }
  }
  void setMode(Mode mode) { this->mode = mode; }

  Mode getMode() { return mode; }

  String getStringMode() {
    switch (mode) {
    case NBK_MODE:
      return STRING_NBK_MODE;
    case RECT_MODE:
      return activeMode->getName();
    default:
      return STRING_ERR;
    }
  }
  void nextMode() {
    switch (mode) {
    case NBK_MODE:
      setMode(RECT_MODE);
      activeMode = &rect_1;
      break;
    case RECT_MODE:
      if (activeMode == &rect_1) {
        activeMode = &rect_2;
      } else {
        setMode(NBK_MODE);
      }
      break;
    default:
      break;
    }
  }
  void backMode() {
    switch (mode) {
    case NBK_MODE:
      setMode(RECT_MODE);
      activeMode = &rect_2;
      break;
    case RECT_MODE:
      if (activeMode == &rect_2) {
        activeMode = &rect_1;
      } else {
        setMode(NBK_MODE);
      }
      break;
    default:
      break;
    }
  }
  bool modeDelay(uint8_t &i, bool error, uint8_t time = 5) {
    if (error) {
      i++;
      if (i > time) {
        return true;
      }
    } else {
      if (i != 0) {
        i = 0;
      }
    }
    return false;
  }
  void runNBK() {
    if (status == OVERCLOCK) {
      if (temperature.getOutputTemp() > data.nbk_output) {
        if (modeDelay(overclock_delay, true)) {
          setStatus(STABILIZATION);
          buzzer.sing(BUZZER_INFO);
        }

      } else {
        modeDelay(overclock_delay, false);
      }
    }
    if (status == STABILIZATION) {
      if (stab_end <= millis()) {
        setStatus(PROCESS);
        data.pump_speed = 10;
        buzzer.sing(BUZZER_INFO);
      }
    }
    if (status == PROCESS) {
      if (pump.getSpeed() < 5) {
        error_braga++;
        if (error_braga > 30) {
          setStatus(END);
          buzzer.setBuzzerType(BUZZER_END);
          buzzer.setEnabled(true);
        }
      } else {
        if (error_braga != 0) {
          error_braga = 0;
        }
      }
      if (temperature.getBardTemp() < data.nbk_bard - 7) {
        error_bard++;
        if (error_bard > 30) {
          setStatus(ERROR_BARD);
          buzzer.setBuzzerType(BUZZER_ERROR);
          buzzer.setEnabled(true);
        }
      } else {
        if (error_bard != 0) {
          error_bard = 0;
        }
      }
      if (temperature.getOutputTemp() > data.nbk_output + 7) {
        error_output++;
        if (error_output > 30) {
          setStatus(ERROR_BARD);
          buzzer.setBuzzerType(BUZZER_ERROR);
          buzzer.setEnabled(true);
        }
      } else {
        if (error_output != 0) {
          error_output = 0;
        }
      }
      float f = 0.01;
      if (temperature.getBardTemp() > data.nbk_bard + data.nbk_delta ||
          temperature.getOutputTemp() > data.nbk_output) {
        if (data.pump_speed < 30) {
          data.pump_speed += f;
        }
      } else if (temperature.getBardTemp() < data.nbk_bard - data.nbk_delta) {
        if (data.pump_speed > 10) {
          data.pump_speed -= f;
        }
      }
    }
  }
  void runRECT() {
    if (status == OVERCLOCK) {
      if (temperature.getOutputTemp() > activeMode->getData()->rect_output) {
        if (modeDelay(overclock_delay, true, 10)) {
          setStatus(STABILIZATION);
          buzzer.sing(BUZZER_INFO);
        }
      } else {
        modeDelay(overclock_delay, false);
      }
    }
    if (status == STABILIZATION) {
      if (stab_end <= millis()) {
        setStatus(HEAD);
        buzzer.sing(BUZZER_INFO);
      }
    }
    if (status == HEAD) {
      selection_valve_open_time = calculateSelectionValveOpenTime(
          activeMode->getData()->rect_speed_head);
      // if(digitalRead(HEAD_FULL_PIN) == LOW){
      //   if (modeDelay(head_full_delay, true, 5)) {
      //     setStatus(BODY);
      //   }
      // } else {
      //   modeDelay(head_full_delay, false);
      // }
    }
    if (status == BODY || status == TAIL) {
      if (pause_tail) {
        return;
      }
      float delta = status == BODY ? activeMode->getData()->rect_delta
                                   : activeMode->getData()->rect_delta_tail;
      if (selection_valve_open_time == 0 && !pause_body) {
        selection_valve_open_time = calculateSelectionValveOpenTime(
            activeMode->getData()->rect_speed_body);
        real_speed_body = activeMode->getData()->rect_speed_body;
        start_body_temp = temperature.getOutputTemp();
      }
      if (temperature.getOutputTemp() != temp_err &&
          temperature.getOutputTemp() > start_body_temp + delta &&
          !pause_body) {
        if (modeDelay(rect_pause_delay, true, 5)) {
          selection_valve_open_time = 0;
          float f =
              1 -
              (static_cast<float>(activeMode->getData()->rect_speed_reduction) /
               100);
          real_speed_body = real_speed_body * f;
          pause_body = true;
          buzzer.sing(BUZZER_INFO);
          pause_start_time = millis();
        }
      } else {
        modeDelay(rect_pause_delay, false);
      }
      if (pause_body && pause_start_time + RECT_DELTA_PAUSE < millis() &&
          temperature.getOutputTemp() <= start_body_temp + delta) {
        if (modeDelay(rect_cancel_pause_delay, true, 5)) {
          pause_body = false;
          selection_valve_open_time =
              calculateSelectionValveOpenTime(real_speed_body);
          buzzer.sing(BUZZER_INFO);
        }
      } else {
        modeDelay(rect_cancel_pause_delay, false);
      }
      if (temperature.getCubeTemp() > activeMode->getData()->rect_cube_tail &&
          status != TAIL) {
        if (modeDelay(tail_delay, true, 20)) {
          selection_valve_open_time = 0;
          pause_tail = true;
          buzzer.setBuzzerType(BUZZER_END);
          buzzer.setEnabled(true);
        }
      } else {
        modeDelay(tail_delay, false);
      }
      if (temperature.getCubeTemp() > activeMode->getData()->rect_cube_end) {
        if (modeDelay(end_delay, true, 5)) {
          setStatus(END);
          selection_valve_open_time = 0;
          buzzer.setBuzzerType(BUZZER_END);
          buzzer.setEnabled(true);
        }
      } else {
        modeDelay(end_delay, false);
      }
    }
  }
  void run() {
    if (next_run > millis()) {
      return;
    }
    next_run = millis() + 1000;
    if (temperature.getTsaTemp() > data.tsa) {
      if (modeDelay(error_tsa, true, 10)) {
        setStatus(ERROR_TSA);
        buzzer.setBuzzerType(BUZZER_ERROR);
        buzzer.setEnabled(true);
      }
    } else {
      modeDelay(error_tsa, false);
    }
    if (mode == NBK_MODE) {
      runNBK();
    } else if (mode == RECT_MODE) {
      runRECT();
    }
    relayCheck();
  }

  void selectionValveCheck() {
    if (selection_valve_open_time >= SELECTION_VALVE_TIME) {
      if (!relay.isOpenSelectionValve()) {
        relay.openSelectionValve();
      }
      return;
    } else if (selection_valve_open_time == 0) {
      if (relay.isOpenSelectionValve()) {
        relay.closeSelectionValve();
      }
      return;
    }
    if (relay.isOpenSelectionValve()) {
      if (relay.getSelectionValveOpenTime() + selection_valve_open_time <
          millis()) {
        relay.closeSelectionValve();
      }
    } else {
      if (relay.getSelectionValveOpenTime() + SELECTION_VALVE_TIME < millis()) {
        relay.openSelectionValve();
      }
    }
  }

  unsigned long getStabilizationRestTime() {
    if (stab_end < millis() || stab_end == 0) {
      lcd.setCursor(0, 2);
      lcd.print(stab_end);
      return 0;
    }
    unsigned long l = stab_end;
    l = l - millis();
    l = l / 60000;
    return l;
  }
  void setStabilizationRestTime(uint8_t i) {
    stab_end = millis() + (i * 60000);
    stab_end = stab_end + 2000;
  }

  uint16_t getRealSpeedBody() { return real_speed_body; }
  void setRealSpeedBody(uint16_t i) { real_speed_body = i; }
  uint16_t calculateSelectionValveOpenTime(uint16_t speed) {
    if (speed == 0) {
      return 0;
    }
    float t = SELECTION_VALVE_TIME;
    float c = SELECTION_VALVE_COEFF;
    float s = speed;
    float o = SELECTION_VALVE_OPEN_TIME;
    float te = (t / (c / s)) + o;
    if (te > t) {
      te = t;
    }
    return static_cast<uint16_t>(te);
  }
  uint16_t getSelectionValveOpenTime() { return selection_valve_open_time; }
  void updateSpeedBody() {
    selection_valve_open_time =
        calculateSelectionValveOpenTime(activeMode->getData()->rect_speed_body);
  }
  float getStartBodyTemp() { return start_body_temp; }
  void setStartBodyTemp(float f) { start_body_temp = f; }
};
NBK nbk;
class Time {
private:
  unsigned long lastUpdate = 0;
  unsigned long next_update = 0;
  String lastTimeString = "00:00";

public:
  String getTime() {
    if (next_update > millis()) {
      return lastTimeString;
    }
    next_update = millis() + 10000;
    unsigned long time;
    if (nbk.getStartTime() == 0) {
      time = 0;
    } else if (nbk.getStopTime() != 0 &&
               nbk.getStopTime() > nbk.getStartTime()) {
      time = nbk.getStopTime() - nbk.getStartTime();
    } else {
      time = millis() - nbk.getStartTime();
    }
    if (lastUpdate == 0 || lastUpdate + 10000 < millis()) {
      time = time / 1000;
      int h = time / 3600;
      int m = time % 3600 / 60;
      lastTimeString = String(h / 10) + String(h % 10) + ":" + String(m / 10) +
                       String(m % 10);
      lastUpdate = millis();
    }
    return lastTimeString;
  }

  unsigned long getLastUpdate() { return lastUpdate; }
};

Time time;
const uint8_t max_screen = 4;
enum Screen {
  PUMP_SCREEN,
  TEMPERATURES_SCREEN,
  RECT_SCREEN,
  TIME_SCREEN,
  WATT_SCREEN
};
enum Select {
  NONE_SELECT,
  DATA_PUMP_SPEED,
  REAL_PUMP_SPEED,
  FULL_PUMP_LITERS,
  COEFF_PUMP_AND_PWM,

  BARD_TEMP,
  OUTPUT_TEMP,
  TSA_TEMP,
  STATUS,
  MODE,
  DELTA,
  SPEED_HEAD,
  SPEED_BODY,
  SPEED_REDUCTION,

  STABILIZATION_TIME,
  REAL_SPEED_BODY,
  START_BODY_TEMP,
  FULL_TIME,
  SELECTION_VALVE_OPEN_TIME_SELECT,
  WATT,
  END_TEMP,
  END_STATUS
};

/*
25.3L/h 25.4L/h
99.8L 1.33/1023

 17.5 18.3 27.8
 Process   NBK  

 D:1.0 SH:150
 SB:2000 R:10

 S:0 B:1500 78.0
 T:20:00 SV:5000

 W:2500 КТ:91.4
 КС:Разгон
*/
enum PositionType { INT_POSITION, FLOAT_POSITION, STRING_POSITION };
class Position {
public:
  Select select_type;
  PositionType p_type;
  String preffix;
  String ending;
  uint8_t col;
  uint8_t row;
  uint8_t size;
  uint8_t screen;
  bool cursor;
  float last;

  Position(Select select_type, PositionType p_type, String preffix,
           String ending, uint8_t col, uint8_t row, uint8_t size,
           uint8_t screen, bool cursor, float last) {
    this->select_type = select_type;
    this->p_type = p_type;
    this->preffix = preffix;
    this->ending = ending;
    this->col = col;
    this->row = row;
    this->size = size;
    this->screen = screen;
    this->cursor = cursor;
    this->last = last;
  }
};
uint8_t POSITION_SIZE = 22;
Position positions[] = {
    {NONE_SELECT, FLOAT_POSITION, "", "", 0, 0, 0, 0, false, 0},

    {DATA_PUMP_SPEED, FLOAT_POSITION, "", "л/ч", 0, 0, 7, 0, false, 0},
    {REAL_PUMP_SPEED, FLOAT_POSITION, "", "л/ч", 8, 0, 8, 0, false, 0},
    {FULL_PUMP_LITERS, FLOAT_POSITION, "", "л", 0, 1, 5, 0, false, 0},
    {COEFF_PUMP_AND_PWM, FLOAT_POSITION, "", "", 6, 1, 9, 0, false, 0},

    {BARD_TEMP, FLOAT_POSITION, "", "", 1, 0, 4, 1, true, 0},
    {OUTPUT_TEMP, FLOAT_POSITION, "", "", 7, 0, 4, 1, true, 0},
    {TSA_TEMP, FLOAT_POSITION, "", "", 12, 0, 4, 1, true, 0},
    {STATUS, STRING_POSITION, "", "", 1, 1, 9, 1, true, 0},
    {MODE, STRING_POSITION, "", "", 11, 1, 5, 1, true, 0},

    {DELTA, FLOAT_POSITION, "Д:", "", 1, 0, 3, 2, true, 0},
    {SPEED_HEAD, INT_POSITION, "СГ:", "", 7, 0, 3, 2, true, 0},
    {SPEED_BODY, INT_POSITION, "СТ:", "", 1, 1, 4, 2, true, 0},
    {SPEED_REDUCTION, INT_POSITION, "R:", "", 9, 1, 2, 2, true, 0},

    {STABILIZATION_TIME, INT_POSITION, "C:", "", 1, 0, 2, 3, true, 0},
    {REAL_SPEED_BODY, INT_POSITION, "C:", "", 5, 0, 4, 3, true, 0},
    {START_BODY_TEMP, FLOAT_POSITION, "", "", 12, 0, 4, 3, true, 0},
    {FULL_TIME, STRING_POSITION, "В:", "", 1, 1, 4, 3, false, 0},
    {SELECTION_VALVE_OPEN_TIME_SELECT, INT_POSITION, "КВ:", "", 9, 1, 4, 3,
     false, 0},
    {WATT, INT_POSITION, "W:", "", 1, 0, 4, 4, true, 0},
    {END_TEMP, FLOAT_POSITION, "КТ:", "", 8, 0, 4, 4, false, 0},
    {END_STATUS, STRING_POSITION, "КС:", "", 1, 1, 9, 4, false, 0}};

class Display {
private:
  Screen screen = TEMPERATURES_SCREEN;
  unsigned long next_update = 0;
  Select select = NONE_SELECT;
  String cursor = STRING_CURSOR;

  float cutFloat(float f) {
    f *= 10;
    f = floor(f + 0.5);
    f /= 10;
    return f;
  }

public:
  Screen getScreen() { return screen; }
  void nextScreen() {
    uint8_t i = static_cast<uint8_t>(screen);
    i++;
    if (i > max_screen) {
      i = 0;
    }
    screen = static_cast<Screen>(i);
    if (select != NONE_SELECT) {
      select = NONE_SELECT;
    }
    print();
  }
  void print() {
    lcd.clear();
    for (uint8_t i = 0; i < POSITION_SIZE; i++) {
      if (getScreen() == positions[i].screen) {
        unitPrint(positions[i].select_type, true);
      }
    }
    next_update = millis() + 300;
  }
  void update(bool f = false) {
    if (f == false && next_update > millis()) {
      return;
    }
    for (uint8_t i = 0; i < POSITION_SIZE; i++) {
      if (getScreen() == positions[i].screen) {
        unitPrint(positions[i].select_type, false, true);
      }
    }
    next_update = millis() + 300;
  }
  Position getPosition(Select s) {
    for (uint8_t i = 0; i < POSITION_SIZE; i++) {
      if (positions[i].select_type == s) {
        return positions[i];
      }
    }
    return positions[0];
  }
  Position *getPositionPointer(Select s) {
    for (uint8_t i = 0; i < POSITION_SIZE; i++) {
      if (positions[i].select_type == s) {
        return &positions[i];
      }
    }
    return &positions[0];
  }
  Position getNextPosition(Position from, bool next) {
    Position temp = positions[0];
    if (next) {
      for (uint8_t i = 1; i < POSITION_SIZE; i++) {
        Position p = positions[i];
        if (screen == p.screen) {
          if (p.cursor) {
            if ((p.col > from.col && p.row == from.row) || (p.row > from.row) ||
                from.select_type == NONE_SELECT) {
              return p;
            }
          }
        }
      }
    } else {
      for (uint8_t i = POSITION_SIZE - 1; i > 0; i--) {
        Position p = positions[i];
        if (screen == p.screen) {
          if (p.cursor) {
            if ((p.col < from.col && p.row == from.row) || (p.row < from.row) ||
                from.select_type == NONE_SELECT) {
              return p;
            }
          }
        }
      }
    }
    return temp;
  }

  void updateCursor(bool remove) {
    if (select == NONE_SELECT) {
      return;
    }
    uint8_t c = 0;
    Position p = getPosition(select);
    c = p.col;
    if (c > 0) {
      c--;
    }
    lcd.setCursor(c, p.row);
    if (remove) {
      lcd.print(" ");
    } else {
      lcd.print(cursor);
    }
  }
  void unitPrint(Select select = NONE_SELECT, bool full = false,
                 bool changed = false) {
    if (select == NONE_SELECT) {
      select = this->select;
    }
    if (select == NONE_SELECT) {
      return;
    }
    float value = 0;
    String stringValue = "";
    Position *p = getPositionPointer(select);
    switch (select) {
    case DATA_PUMP_SPEED:
      value = data.pump_speed;
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case REAL_PUMP_SPEED:
      value = pump.getSpeed();
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case FULL_PUMP_LITERS:
      value = pump.getLiters();
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case COEFF_PUMP_AND_PWM:
      stringValue = String(data.pump_coeff, 2) + "/" + String(pump.p);
      value = data.pump_coeff + pump.p;
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case BARD_TEMP:
      if (this->select != BARD_TEMP) {
        value = temperature.getBardTemp();
        if (value == temp_err) {
          stringValue = STRING_ERR;
        }
      } else if (nbk.getMode() == NBK_MODE) {
        value = data.nbk_bard;
      } else if (nbk.getMode() == RECT_MODE) {
        if (nbk.getStatus() != TAIL) {
          value = activeMode->getData()->rect_cube_tail;
        } else {
          value = activeMode->getData()->rect_cube_end;
        }
      }
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case OUTPUT_TEMP:
      if (this->select != OUTPUT_TEMP) {
        value = temperature.getOutputTemp();
        if (value == temp_err) {
          stringValue = STRING_ERR;
        }
      } else if (nbk.getMode() == NBK_MODE) {
        value = data.nbk_output;
      } else if (nbk.getMode() == RECT_MODE) {
        value = activeMode->getData()->rect_output;
      }
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case TSA_TEMP:
      if (this->select != TSA_TEMP) {
        value = temperature.getTsaTemp();
        if (value == temp_err) {
          stringValue = STRING_ERR;
        }
      } else if (nbk.getMode() == NBK_MODE || nbk.getMode() == RECT_MODE) {
        value = data.tsa;
      }
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case STATUS:
      stringValue = nbk.getStringStatus();
      if (changed && static_cast<Status>(p->last) == nbk.getStatus()) {
        return;
      }
      p->last = static_cast<float>(nbk.getStatus());
      break;
    case MODE:
      stringValue = nbk.getStringMode();
      if (changed && static_cast<Mode>(p->last) == nbk.getMode()) {
        return;
      }
      p->last = static_cast<float>(nbk.getMode());
      break;
    case DELTA:
      if (nbk.getMode() == NBK_MODE) {
        value = data.nbk_delta;
      } else if (nbk.getMode() == RECT_MODE) {
        if (nbk.getStatus() != TAIL) {
          value = activeMode->getData()->rect_delta;
        } else {
          value = activeMode->getData()->rect_delta_tail;
        }
      }
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case SPEED_HEAD:
      value = activeMode->getData()->rect_speed_head;
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case SPEED_BODY:
      value = activeMode->getData()->rect_speed_body;
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case SPEED_REDUCTION:
      value = activeMode->getData()->rect_speed_reduction;
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case STABILIZATION_TIME:
      value = nbk.getStabilizationRestTime();
      if (changed && p->last == value) {
        return;
      }
      p->last = nbk.getStabilizationRestTime();
      break;
    case REAL_SPEED_BODY:
      value = nbk.getRealSpeedBody();
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case START_BODY_TEMP:
      value = nbk.getStartBodyTemp();
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case FULL_TIME:
      stringValue = time.getTime();
      if (changed && p->last == time.getLastUpdate()) {
        return;
      }
      p->last = time.getLastUpdate();
      break;
    case SELECTION_VALVE_OPEN_TIME_SELECT:
      value = nbk.getSelectionValveOpenTime();
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case WATT:
      if (nbk.getMode() == NBK_MODE) {
        value = data.nbk_watt;
      } else if (nbk.getMode() == RECT_MODE) {
        value = activeMode->getData()->rect_watt;
      }
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case END_TEMP:
      value = end_data.end_cube_temp;
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    case END_STATUS:
      stringValue = nbk.getStringStatus(end_data.end_status);
      if (changed && p->last == value) {
        return;
      }
      p->last = value;
      break;
    default:
      return;
    }
    uint8_t c = p->col;

    if (full) {
      lcd.setCursor(c, p->row);
      lcd.print(p->preffix);
    }
    c = c + p->preffix.length();
    lcd.setCursor(c, p->row);
    for (uint8_t i = 0; i < p->size; i++) {
      lcd.print(" ");
    }
    lcd.setCursor(c, p->row);
    if (p->p_type == STRING_POSITION || stringValue.length() > 0) {
      lcd.print(stringValue);
      lcd.setCursor(c, p->row);
    } else {
      lcd.print(String(value, static_cast<int>(p->p_type)));
    }
    if (p->ending.length() != 0) {
      lcd.print(p->ending);
    }
  }
  void updateSelect(bool next) {
    Position from = getPosition(select);
    removeCursor();
    Position p = getNextPosition(from, next);
    select = p.select_type;
    if (select == NONE_SELECT) {
      return;
    }
    printCursor();
    unitPrint(select);
  }
  void nextSelect() { updateSelect(true); }

  void backSelect() { updateSelect(false); }

  Select getSelect() { return select; }

  void removeCursor() { updateCursor(true); }
  void printCursor() { updateCursor(false); }

  void changeSelectValue(int l, bool up) {
    if (select == NONE_SELECT) {
      return;
    }
    float f = l > 0 ? 0.5 : 0.1;
    int i = l > 0 ? 5 : 1;
    if (up == false) {
      f = -f;
      i = -i;
    }
    switch (select) {
    case BARD_TEMP:
      if (nbk.getMode() == NBK_MODE) {
        data.nbk_bard = data.nbk_bard + f;
      } else if (nbk.getMode() == RECT_MODE) {
        if (nbk.getStatus() == TAIL) {
          activeMode->getData()->rect_cube_end =
              activeMode->getData()->rect_cube_end + f;
        } else {
          activeMode->getData()->rect_cube_tail =
              activeMode->getData()->rect_cube_tail + f;
        }
      }
      eepromHandler.saveTask();
      // updateSelect();
      break;
    case OUTPUT_TEMP:
      if (nbk.getMode() == NBK_MODE) {
        data.nbk_output = data.nbk_output + f;
      } else if (nbk.getMode() == RECT_MODE) {
        activeMode->getData()->rect_output =
            activeMode->getData()->rect_output + f;
      }
      eepromHandler.saveTask();
      break;
    case TSA_TEMP:
      data.tsa = data.tsa + f;
      eepromHandler.saveTask();
      break;
    case STATUS:
      up ? nbk.nextStatus() : nbk.backStatus();
      break;
    case MODE:
      up ? nbk.nextMode() : nbk.backMode();
      break;
    case DELTA:
      if (nbk.getMode() == NBK_MODE) {
        data.nbk_delta = data.nbk_delta + f;
      } else if (nbk.getMode() == RECT_MODE) {
        if (nbk.getStatus() != TAIL) {
          activeMode->getData()->rect_delta =
              activeMode->getData()->rect_delta + f;
        } else {
          activeMode->getData()->rect_delta_tail =
              activeMode->getData()->rect_delta_tail + f;
        }
      }
      eepromHandler.saveTask();
      break;
    case SPEED_HEAD:
      activeMode->getData()->rect_speed_head =
          activeMode->getData()->rect_speed_head + i;
      eepromHandler.saveTask();
      break;
    case SPEED_BODY:
      activeMode->getData()->rect_speed_body =
          activeMode->getData()->rect_speed_body + i;
      nbk.updateSpeedBody();
      eepromHandler.saveTask();
      break;
    case SPEED_REDUCTION:
      activeMode->getData()->rect_speed_reduction =
          activeMode->getData()->rect_speed_reduction + i;
      eepromHandler.saveTask();
      break;
    case STABILIZATION_TIME:
      nbk.setStabilizationRestTime(nbk.getStabilizationRestTime() + i);
      break;
    case REAL_SPEED_BODY:
      nbk.setRealSpeedBody(nbk.getRealSpeedBody() + i);
      break;
    case START_BODY_TEMP:
      nbk.setStartBodyTemp(nbk.getStartBodyTemp() + f);
      break;
    case WATT:
      if (nbk.getMode() == NBK_MODE) {
        data.nbk_watt += i;
      } else if (nbk.getMode() == RECT_MODE) {
        activeMode->getData()->rect_watt += i;
      }
      eepromHandler.saveTask();
    default:
      return;
    }
    update(true);
  }
};
Display display;

enum Button {
  UP = 0,
  DOWN = 1,
  RIGHT = 2,
  LEFT = 3,
  NEXT_SCREEN = 4,
  NONE = 9
};

class Keyboard {
private:
  unsigned long delay = 500;
  unsigned long update = 50;
  uint8_t switch_speed = 30;
  unsigned long next_update = 0;
  unsigned long nextPress = 0;
  unsigned int l;

  Button lastButton = NONE;

  Button getPressedButton() {
    for (uint8_t i = 0; i < 5; i++) {
      if (digitalRead(keyboard_pins[i]) == LOW) {
        return static_cast<Button>(i);
      }
    }
    return NONE;
  }

  void calculateDelay(Button button) {
    if (button != lastButton) {
      l = 0;
    } else {
      l++;
    }
    if (l < 3) {
      nextPress = millis() + delay;
    } else if (l < 5) {
      nextPress = millis() + (delay / 2);
    } else if (l < 20) {
      nextPress = millis() + (delay / 5);
    } else if (l < switch_speed) {
      nextPress = millis() + (delay / 10);
    }
  }
  void pressButton(Button button) {
    if (buzzer.isEnabled()) {
      buzzer.setEnabled(false);
      buzzer.setBuzzerType(BuzzerType::BUZZER_NONE);
    }
    buzzer.sing(BUZZER_BUTTON);
    switch (button) {
    case UP:
      if (display.getScreen() == PUMP_SCREEN) {
        if (pump.manual) {
          pump.p = l > switch_speed ? pump.p + 5 : pump.p + 1;
          if (pump.p > PWM_MAX) {
            pump.p = PWM_MAX;
          }
          pump.pwm();
        } else {
          data.pump_speed =
              l > switch_speed ? data.pump_speed + 0.5 : data.pump_speed + 0.1;
          eepromHandler.saveTask();
        }
      } else {
        display.changeSelectValue(l - switch_speed, true);
      }
      break;
    case DOWN:
      if (display.getScreen() == PUMP_SCREEN) {
        if (pump.manual) {
          if (l > switch_speed) {
            if (pump.p < 5) {
              pump.p = 0;
            } else {
              pump.p -= 5;
            }
          } else {
            if (pump.p != 0) {
              pump.p -= 1;
            }
          }
          pump.pwm();
        } else {
          data.pump_speed =
              l > switch_speed ? data.pump_speed - 0.5 : data.pump_speed - 0.1;
          if (data.pump_speed < 0) {
            data.pump_speed = 0;
          }
          eepromHandler.saveTask();
        }
      } else {
        display.changeSelectValue(l - switch_speed, false);
      }
      break;
    case RIGHT:
      if (display.getScreen() == PUMP_SCREEN) {
        pump.manual = !pump.manual;
      } else {
        display.nextSelect();
      }
      break;
    case LEFT:
      if (display.getScreen() == PUMP_SCREEN) {
        if (!pump.calibration && l > switch_speed) {
          pump.calibrate();
          l = 0;
          nextPress = millis() + 2000;
        } else if (pump.calibration) {
          pump.calibrate();
          eepromHandler.saveTask();
        }
      } else {
        display.backSelect();
      }
      break;
    case NEXT_SCREEN:
      display.nextScreen();
      break;
    default:
      break;
      return;
    }
  }

public:
  void run() {
    if (next_update > millis()) {
      return;
    }
    next_update = millis() + update;
    Button button = getPressedButton();
    if (button == NONE && lastButton != NONE) {
      lastButton = NONE;
      l = 0;
      return;
    }
    if (button == NONE) {
      return;
    }
    if (button != lastButton || nextPress < millis()) {
      lastButton = button;
      calculateDelay(button);
      pressButton(button);
      display.update(true);
      return;
    }
  }

  bool isPressed() { return lastButton != NONE; }
};

Keyboard keyboard;
void pulse() { pump.pulse(); }

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(SELECTION_VALVE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TENG_ONE_PIN, OUTPUT);
  pinMode(TENG_TWO_PIN, OUTPUT);
  pinMode(FLOW_PIN, INPUT);
  pinMode(HEAD_FULL_PIN, INPUT_PULLUP);
  pinMode(COOLER_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulse, RISING);
  sei();
  for (uint8_t i = 0; i < 5; i++) {
    pinMode(keyboard_pins[i], INPUT_PULLUP);
  }
  for (uint8_t i = 0; i < PUMP_CONTROL_SECOND; i++) {
    tick[i] = 0;
  }
  TCCR1A = TCCR1A & 0xe0 | 3;
  TCCR1B = TCCR1B & 0xe0 | 0x09;
  // TCCR1A = TCCR1A & 0xe0 | 3;
  // TCCR1B = TCCR1B & 0xe0 | 0x0a;
  eepromHandler.load();
  pump.pwm();
  relay.setup();
  display.print();
}
unsigned long max = 0;
unsigned long last = 0;
unsigned long next_ent_update_time = 0;

void endDataUpdate() {
  end_data.end_cube_temp = temperature.getCubeTemp();
  end_data.end_status = static_cast<uint8_t>(nbk.getStatus());
  eepromHandler.saveEndDataTask();
  next_ent_update_time = next_ent_update_time + END_DATA_SAVE_TIME;
}
void loop() {
  keyboard.run();
  pump.writePulses();
  if (!pump.manual) {
    pump.calculate();
  }
  display.update();
  nbk.run();
  nbk.selectionValveCheck();
  buzzer.sing();
  eepromHandler.check();
  time.getTime();
  packetHandler.send();
  packetHandler.take();
  if (packetHandler.getServoLastAnswerTime() != last) {
    unsigned long difference = packetHandler.getServoLastAnswerTime() - last;
    if (difference > max) {
      max = difference;
    }
    last = packetHandler.getServoLastAnswerTime();
  }
  if (packetHandler.getServoLastAnswerTime() + 60000 < millis()) {
    if (nbk.getStatus() != OFF && nbk.getStatus() != END) {
      nbk.setStatus(ERROR_SERVO);
      buzzer.setBuzzerType(BUZZER_ERROR);
      buzzer.setEnabled(true);
    }
  }
  if (nbk.getStatus() != OFF && next_ent_update_time < millis()) {
    endDataUpdate();
  }
}