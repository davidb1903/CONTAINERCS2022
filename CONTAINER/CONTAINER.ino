
#include "constDef.h"
#include "libContainer.h"

Preferences preferences;
Adafruit_BMP280 bmp; // I2C
RTC_DS3231 rtc;
Servo servo_paracaidas;
Servo servo_payload;
SoftwareSerial SerialGPS(32, 33);
TinyGPSPlus gps;
TaskHandle_t TaskHandle_Serial; // handler for TasksSensors
QueueHandle_t arrayQueue;
//
void TaskGPS(void *pvParameters);
void init_Interfaces(void);
void validation_comand(void);
//
unsigned int state_f;
unsigned int state_p;
float pressure;
float sensorRead[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
String states[6] = {"LAUNCH_WAIT", "ASCENT", "DESCENT", "SP_RELEASED", "TP_RELEASED", "LANDED"};
String statesPay[4] = {"STANDBY", "RELEASED", "TARGET_DETECTION", "LANDED"};
String cmd[11] = {"CXON", "CXOFF", "ST", "SIMENABLE", "SIMACTIVATE", "SIMDISABLE", "SIMPRES", "ACTIVATEBUZZER", "FPMLIB", "TPMLIB","PAYTEL"};
float altitude_ref = 0;
float alt_buffer;
float altitude_SM = 0;
const float p0 = 1013.25;
float temp = 0;
float altitude = 0;
float altitude_buffer = 0;
long container_countp = 0;
long payload_countp = 0;
String ECHO = "";
char MODE = 'F';
int tel_status = 0;
//
void setup() {
  // put your setup code here, to run once:
  init_Interfaces();
  if (state_f == 0) {
    while (tel_status == 0) {
      if (Serial.available() > 0 ) {
        char cmd_char = Serial.read();
        validation_comand(cmd_char);
        if (tel_status == 1) {
          break;
        }
      }
    }
  }
  else{
    xTaskCreatePinnedToCore(TaskSerial, "PrintSerial", 2048, NULL, 10, &TaskHandle_Serial, 1);
    }
}
void loop() {
  sensorRead[2] = bmp.readTemperature();
  alt_buffer = sensorRead[1];
  if (MODE == 'F') {
    sensorRead[1] = bmp.readAltitude(1013.25) - altitude_ref;
  }
  else {
    sensorRead[1] = altitude_SM - altitude_ref;
  }
  sensorRead[3] = float(analogRead(34) * 0.00161172161);
  DateTime now = rtc.now();
  sensorRead[11] = now.hour();
  sensorRead[12] = now.minute();
  sensorRead[13] = now.second();
  // }
  xQueueSend(arrayQueue, &sensorRead, portMAX_DELAY);
  switch (state_f) {
    case 0:
      if (sensorRead[1] > 5) { //Condicional Ascenso}
        state_f = 1;
        preferences.begin("CONTAINER", false);
        preferences.putUInt("fstate", state_f);
        preferences.end();
      }
      break;
    case 1:
      if (sensorRead[1]- alt_buffer<abs(4)) { //Condicional Descenso
        state_f = 2;
        preferences.begin("CONTAINER", false);
        preferences.putUInt("fstate", state_f);
        preferences.end();
      }
      break;
    case 2:
      if (sensorRead[1] < 400) { //Condicional Segundo paracaidas
        state_f = 3;
        servo_paracaidas.write(22);//
        preferences.begin("CONTAINER", false);
        preferences.putUInt("fstate", state_f);
        preferences.end();
      }
      break;
    case 3:
      if (sensorRead[1] < 310) { //Condicional Payload
        state_f = 4;
        servo_payload.write(180);
        Serial1.print('A');
        xTaskCreate(TaskPayloadLib, "TaskPayloadLib", 2048, NULL, 12, NULL);
        preferences.begin("CONTAINER", false);
        preferences.putUInt("fstate", state_f);
        preferences.end();
      }
      break;
    case 4:
      while (Serial1.available() > 0 ) {
        char cmd_char = Serial1.read();
        if (cmd_char == 'X') {
          Serial.print("1017");
          Serial.print(',');
          Serial.print(sensorRead[11], 0);
          Serial.print(':');
          Serial.print(sensorRead[12], 0);
          Serial.print(':');
          Serial.print(sensorRead[13], 2);
          Serial.print(',');
          Serial.print(payload_countp++);
          Serial.print(',');
          Serial.print('T');
          Serial.print(',');
        } else {
          Serial.print(cmd_char);
        }
      }
      if (sensorRead[1] < 20) { //Condicional Aterrizaje
        Serial1.print('B');
        Serial1.print('B');
        state_f = 5;
        preferences.begin("CONTAINER", false);
        preferences.putUInt("fstate", state_f);
        preferences.end();
      } 
      break;
    case 5:
      digitalWrite(15, HIGH);
      digitalWrite(PIN_CAMARA, HIGH);
        while (Serial1.available() > 0 ) {
        char cmd_char = Serial1.read();
        if (cmd_char == 'X') {
          Serial.print("1017");
          Serial.print(',');
          Serial.print(sensorRead[11], 0);
          Serial.print(':');
          Serial.print(sensorRead[12], 0);
          Serial.print(':');
          Serial.print(sensorRead[13], 2);
          Serial.print(',');
          Serial.print(payload_countp++);
          Serial.print(',');
          Serial.print('T');
          Serial.print(',');
        } else {
          Serial.print(cmd_char);
        }
      }
      delay(5000);
      vTaskDelete(TaskHandle_Serial);
      digitalWrite(PIN_CAMARA, LOW);
      vTaskDelete(NULL);
      break;
      
    default:
      break;
  }
  if (Serial.available() > 0 ) {
    char cmd_char = Serial.read();
    validation_comand(cmd_char);
  }
}
void init_Interfaces() {
  //Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, 3, 1); //Container-Ground
  pinMode(15, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(2, OUTPUT);
  Serial.begin(9600, SERIAL_8N1, 16, 17); //Payload-Container
  rtc.begin();
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  servo_paracaidas.attach(27); // attaches the servo on pin 18 to the servo object
  servo_payload.attach(14);
  SerialGPS.begin(9600);
  alt_buffer = bmp.readAltitude(1013.25);
  preferences.begin("CONTAINER", false);
  state_f = preferences.getUInt("fstate", 0);
  ECHO = preferences.getString("EchoS", "CMXON");
  if (state_f == 0 || state_f == 5) {
    preferences.putLong("containerc", 0);
    preferences.putLong("payloadc", 0);
    preferences.putFloat("altref", alt_buffer);
    if (state_f == 5) {
      state_f = 0;
      preferences.putUInt("fstate", state_f);
    }
  }

  altitude_ref = preferences.getFloat("altref", 0);
  container_countp = preferences.getLong("containerc", 0);
  payload_countp = preferences.getLong("payloadc", 0);
  preferences.end();
  arrayQueue = xQueueCreate(10, sizeof(int)); //Queue item size
  xTaskCreatePinnedToCore(TaskGPS, "TaskGPS", 2048, NULL, 8, NULL, 1);
  servo_paracaidas.write(0);//
  servo_payload.write(0);//
  /////////
}
void validation_comand(char cmd_char) {
  int cmd_in = 0;
  switch (cmd_char) {
    case 'A':  //Activar telemetría
        tel_status = 1;
        cmd_in = 0;
        xTaskCreatePinnedToCore(TaskSerial, "PrintSerial", 2048, NULL, 10, &TaskHandle_Serial, 1);
      break;
    case 'B':
            tel_status = 0;
        Serial1.print('B');
        cmd_in = 1;
        vTaskDelete(TaskHandle_Serial);
    break;

    case 'D':     // SIMENABLE
          cmd_in=10;
      Serial1.print('B');
      Serial1.print('B');
      //MODE = 'S';
      //cmd_in = 3;
      //        if (tel_status==0)
      //         xTaskCreatePinnedToCore(TaskSerial, "PrintSerial", 2048, NULL, 10, &TaskHandle_Serial, 1);
      break;

    case 'E':  //SIMACTIVATE
      tel_status = 1;
      cmd_in = 4;
      xTaskCreatePinnedToCore(TaskSerial, "PrintSerial", 2048, NULL, 10, &TaskHandle_Serial, 1);
      break;

    case 'F': //SIMDISABLE
      tel_status = 0;
      MODE = 'F';
      cmd_in = 5;
      vTaskDelete(TaskHandle_Serial);
      break;

    case 'G': {
        cmd_in = 6;
        float pressureSM = Serial.parseFloat();
        float a = pow(pressureSM / p0, 1 / 5.255);
        altitude_SM = float(44330 * (1 - a));
      }
      break;

    case 'H':  //FLYBUZZER
      cmd_in = 7;
      digitalWrite(15, HIGH);
      break;

    case 'I': // FPMLIB
      cmd_in = 8;
      servo_payload.write(180);
      break;
    case 'J': //TPMLIB
      cmd_in = 9;
      servo_paracaidas.write(22);
      break;
    case 'P':
      cmd_in=10;
      Serial1.print('A');
      Serial1.print('A');
      /*xTaskCreate(TaskPayloadLib, "TaskPayloadLib", 2048, NULL, 12, NULL);*/
      break;
    default:
      servo_paracaidas.write(0);
      break;
  }
  ECHO = cmd[cmd_in];
  preferences.begin("CONTAINER", false);
  preferences.putString("EchoS", ECHO);
  preferences.end();
}
void TaskSerial(void *pvParameters) {
  (void) pvParameters;
  TickType_t lasttick;
  lasttick = xTaskGetTickCount();

  for (;;) {
    if (xQueueReceive(arrayQueue, &sensorRead, portMAX_DELAY) == pdPASS ) {
      Serial.print("1017");
      Serial.print(',');
      Serial.print(sensorRead[11], 0);
      Serial.print(':');
      Serial.print(sensorRead[12], 0);
      Serial.print(':');
      Serial.print(sensorRead[13], 2);
      Serial.print(',');

      Serial.print(container_countp++);
      Serial.print(',');
      Serial.print('C');
      Serial.print(',');
      Serial.print(MODE);
      Serial.print(',');
      if (state_f < 4)
        Serial.print('N');
      else
        Serial.print('R');
      Serial.print(',');
      Serial.print(sensorRead[1], 1);
      Serial.print(',');
      Serial.print(sensorRead[2], 1);                   // Display the results
      Serial.print(',');
      Serial.print(sensorRead[3]);                    // Display the results
      Serial.print(',');
      sensorRead[4] = gps.time.hour();
      Serial.print(sensorRead[4], 0);
      Serial.print(':');
      sensorRead[5] = gps.time.minute();
      Serial.print(sensorRead[5], 0);
      Serial.print(':');
      sensorRead[6] = gps.time.second() + 1;
      Serial.print(sensorRead[6], 0);
      Serial.print(',');
      sensorRead[7] = gps.location.lat();
      Serial.print(sensorRead[7], 4);
      Serial.print(',');
      sensorRead[8] = gps.location.lng();
      Serial.print(sensorRead[8], 4);
      Serial.print(',');
      sensorRead[9] = gps.altitude.meters();
      Serial.print(sensorRead[9], 1);
      Serial.print(',');
      sensorRead[10] = gps.satellites.value();
      Serial.print(sensorRead[10], 0);
      Serial.print(',');
      Serial.print(states[state_f]);
      Serial.print(',');
      Serial.println(ECHO);
      vTaskDelayUntil(&lasttick, 1000 / portTICK_RATE_MS);
    }
  }
}
void TaskGPS(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (SerialGPS.available()) {
      gps.encode(SerialGPS.read());
    }
    vTaskDelay(1);
  }
}
void TaskPayloadLib(void *pvParameters) {
  state_p = 1;
  TickType_t lasttick;
  lasttick = xTaskGetTickCount();
  (void) pvParameters;
  Serial1.print('A');
    digitalWrite(PIN_CAMARA, HIGH);
  vTaskDelayUntil(&lasttick, 3000 / portTICK_RATE_MS);
  digitalWrite(PIN_CAMARA, LOW);
  vTaskDelayUntil(&lasttick, 5000 / portTICK_RATE_MS);
  digitalWrite(PIN_CAMARA, HIGH);
  vTaskDelayUntil(&lasttick, 1000 / portTICK_RATE_MS);
  digitalWrite(PIN_CAMARA, LOW);
  //
  vTaskDelayUntil(&lasttick, 1000 / portTICK_RATE_MS);
  //
  digitalWrite(PIN_MOTORD, LOW);
  digitalWrite(PIN_MOTORI, HIGH);
  //
  vTaskDelayUntil(&lasttick, 5000 / portTICK_RATE_MS);
  digitalWrite(PIN_MOTORD, LOW);
  digitalWrite(PIN_MOTORI, LOW);

  Serial1.print('C');
  Serial1.print('C');
  vTaskDelete(NULL);
}
/*
//Función para guardar los datos en una microSD
void containerSDCard() {
  dataMessage = String("1017") + "," + String(sensorRead[11]) + ":" + String(sensorRead[12]) + ":" + String(sensorRead[13]) + "," + String(container_countp++) 
  + "," + String("C") + "," + String(MODE) + "," + String(sp1) + "," + String(sensorRead[1]) 
  + "," + String(sensorRead[2]) + "," + String(sensorRead[3] )+ "," + String(sensorRead[4]) + ":" + String(sensorRead[5]) + ":" + String(sensorRead[6])
  + "," + String(sensorRead[7]) + "," + String(sensorRead[8]) + "," + String(sensorRead[9]) + "," + String(sensorRead[10]) + "," + String(states[state_f])
  + "," + String(CMD_ECHO) + "\r\n";
  appendFile(SD, "", dataMessage.c_str());
}
/*
//Función para crear un archivo tipo .txt
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("%s\n", path);
  File file = fs.open(path, FILE_WRITE);
  file.close();
}

//Función requerida para escribir sobre un archivo ya existente
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("%s\n", path);
  File file = fs.open(path, FILE_APPEND);
  file.close();
}*/
