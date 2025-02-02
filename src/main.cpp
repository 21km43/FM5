#include <Arduino.h>
#include <M5CoreS3.h>
#include <SoftwareSerial.h>
#define sensor_t sensor_t_
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#undef sensor_t
#include <M5_IMU_PRO.h>
#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>

SoftwareSerial GPSSerial;
SoftwareSerial ALTSerial;

constexpr char SSID[] = "SSID";
constexpr char PASSPHRASE[] = "PASSWORD";

const IPAddress localIP(192, 168, 1, 41);  // 自身のIPアドレス
const IPAddress gateway(192, 168, 1, 0);  // ゲートウェイ
const IPAddress subnet(255, 255, 255, 0); // サブネットマスク

constexpr int GPS_RX = 6, GPS_TX = 7;
constexpr int ALT_RX = 8, ALT_TX = 9;
constexpr int RPM_PIN = 10;
constexpr int TACHO_PIN[2] = {1, 2};
constexpr int SD_SPI_SCK_PIN = 36;
constexpr int SD_SPI_MISO_PIN = 35;
constexpr int SD_SPI_MOSI_PIN = 37;
constexpr int SD_SPI_CS_PIN = 4;

float rudder_rotation = 0;
float elevator_rotation = 0;

#pragma region OTA
void ota_handle(void *parameter)
{
  for (;;)
  {
    ArduinoOTA.handle();
    delay(1000);
  }
}

void setupOTA(const char *nameprefix, const char *ssid, const char *password)
{
  // Configure the hostname
  uint16_t maxlen = strlen(nameprefix) + 7;
  char *fullhostname = new char[maxlen];
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(fullhostname, maxlen, "%s-%02x%02x%02x", nameprefix, mac[3], mac[4], mac[5]);
  ArduinoOTA.setHostname(fullhostname);
  delete[] fullhostname;

  // Configure and start the WiFi station
  WiFi.mode(WIFI_STA);
  WiFi.config(localIP, gateway, subnet);
  WiFi.begin(ssid, password);

  // AP mode
  // WiFi.mode(WIFI_AP);
  // WiFi.softAPConfig(localIP, gateway, subnet);
  // WiFi.softAP(fullhostname, password);

  ArduinoOTA.onStart([]()
                     {
	//NOTE: make .detach() here for all functions called by Ticker.h library - not to interrupt transfer process in any way.
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type); });

  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("\nAuth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("\nBegin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("\nConnect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("\nReceive Failed");
    else if (error == OTA_END_ERROR) Serial.println("\nEnd Failed"); });

  ArduinoOTA.begin();

  Serial.println("OTA Initialized");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  xTaskCreate(
      ota_handle,   /* Task function. */
      "OTA_HANDLE", /* String with name of task. */
      10000,        /* Stack size in bytes. */
      NULL,         /* Parameter passed as input of the task */
      1,            /* Priority of the task. */
      NULL);        /* Task handle. */
}
#pragma endregion

#pragma region BMP280
// 気圧計による気圧、温度の測定及び高度の計算
constexpr int BMP280_SENSOR_ADDR = 0x76;
Adafruit_BMP280 bmp(&Wire1);
float ground_pressure = 1013.25f;
float temperature = 0;
float pressure = 0;
float bmp_altitude = 0;

void InitBMP280()
{
  if (!bmp.begin(BMP280_SENSOR_ADDR))
  {
    Serial.println("BMP280 init failed!");
    return;
  }
  Serial.println("BMP280 OK!");

  delay(100);

  ground_pressure = bmp.readPressure();
}
void GetBMP280()
{
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  bmp_altitude = bmp.readAltitude(ground_pressure);
}
#pragma endregion

#pragma region BMI270
// 6軸+3軸センサによる姿勢角の計算
constexpr int BIM270_SENSOR_ADDR = 0x68;
constexpr int AHRS_SAMPLING = 100;

BMI270::BMI270 bmi270;
Madgwick ahrs_madgwick_6dof, ahrs_madgwick_9dof;
Mahony ahrs_mahony_6dof, ahrs_mahony_9dof;

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
int16_t magX, magY, magZ;

float roll_mad6 = 0, pitch_mad6 = 0, yaw_mad6 = 0;
float roll_mad9 = 0, pitch_mad9 = 0, yaw_mad9 = 0;
float roll_mah6 = 0, pitch_mah6 = 0, yaw_mah6 = 0;
float roll_mah9 = 0, pitch_mah9 = 0, yaw_mah9 = 0;

void ahrs_task(void *pvParameters)
{
  while (1)
  {
    ahrs_madgwick_6dof.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
    ahrs_madgwick_9dof.update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ);
    ahrs_mahony_6dof.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
    ahrs_mahony_9dof.update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ);

    roll_mad6 = ahrs_madgwick_6dof.getRoll();
    pitch_mad6 = ahrs_madgwick_6dof.getPitch();
    yaw_mad6 = ahrs_madgwick_6dof.getYaw();
    roll_mad9 = ahrs_madgwick_9dof.getRoll();
    pitch_mad9 = ahrs_madgwick_9dof.getPitch();
    yaw_mad9 = ahrs_madgwick_9dof.getYaw();
    roll_mah6 = ahrs_mahony_6dof.getRoll();
    pitch_mah6 = ahrs_mahony_6dof.getPitch();
    yaw_mah6 = ahrs_mahony_6dof.getYaw();
    roll_mah9 = ahrs_mahony_9dof.getRoll();
    pitch_mah9 = ahrs_mahony_9dof.getPitch();
    yaw_mah9 = ahrs_mahony_9dof.getYaw();

    delay(AHRS_SAMPLING / 4);
  }
}

void InitBMI270()
{
  if (!bmi270.init(I2C_NUM_1, BIM270_SENSOR_ADDR))
  {
    Serial.println("Failed to find BMI270");
    return;
  }

  ahrs_madgwick_6dof.begin(AHRS_SAMPLING);
  ahrs_madgwick_9dof.begin(AHRS_SAMPLING);
  ahrs_mahony_6dof.begin(AHRS_SAMPLING);
  ahrs_mahony_9dof.begin(AHRS_SAMPLING);

  xTaskCreate(ahrs_task, "ahrs_task", 2048, NULL, 1, NULL);

  Serial.println("BMI270 OK!");
}
void GetBMI270()
{
  bmi270.readAcceleration(accelX, accelY, accelZ);
  bmi270.readGyroscope(gyroX, gyroY, gyroZ);
  bmi270.readMagneticField(magX, magY, magZ);
}
#pragma endregion

#pragma region GPS
// GPSの測定
TinyGPSPlus gps;
double gps_latitude = 0;  // 緯度（小数第9位まで）
double gps_longitude = 0; // 経度（小数第9位まで）
uint16_t gps_year = 0;    // 西暦
uint8_t gps_month = 0;    // 月
uint8_t gps_day = 0;      // 日
uint8_t gps_hour = 0;     // 時
uint8_t gps_minute = 0;   // 分
uint8_t gps_second = 0;   // 秒
double gps_altitude = 0;  // 高度 メートル単位
double gps_course = 0;    // 進行方向(deg)
double gps_speed = 0;     // 対地速度(m/s) 精度は高くないので参考程度に

void InitGPS()
{
  GPSSerial.begin(9600, SWSERIAL_8N1, GPS_RX, GPS_TX);
}
void GetGPS()
{
  while (GPSSerial.available() > 0)
  {
    gps.encode(GPSSerial.read());
    if (gps.location.isUpdated())
    {
      gps_latitude = gps.location.lat();
      gps_longitude = gps.location.lng();
      gps_year = gps.date.year();
      gps_month = gps.date.month();
      gps_day = gps.date.day();
      gps_hour = (gps.time.hour() + 9) % 24; // 時差を考慮すること
      gps_minute = gps.time.minute();
      gps_second = gps.time.second();
      gps_altitude = gps.altitude.meters(); // 高度 メートル単位
      gps_course = gps.course.deg();        // 進行方向(deg)
      gps_speed = gps.speed.mps();
    }
  }
}
#pragma endregion

#pragma region ALTITUDE
// 超音波センサによる高度計のコード

/*
https://files.seeedstudio.com/wiki/RS485_Ultrasonic_level_Sensor/RS485-750cm-Ultrasonic-Level-Sensor.pdf
1. Read the calculated value of distance:
Command: 01 03 01 00 00 01 85 F6
Return: 01 03 02 02 F2 38 A1
Description: The slave address is 0x01, the calculated value of distance is 0x02F2, convert to decimal is 754, the distance
value = 754mm
2. Read the real-time distance value:
Command: 01 03 01 01 00 01 D4 36
Return: 01 03 02 02 EF F8 A8
Description: The slave address is 0x01, the real-time distance value is 0x02EF, convert to decimal is 754, the distance
value = 751mm
3. Read the temperature value:
Command: 01 03 01 02 00 01 24 36
Return: 01 03 02 01 2C B8 09
Description: The slave address is 0x01, the temperature is 0x012C, convert to decimal is 300, the temperature
value = 30.0℃
4. Modify the slave address:
Command: 01 06 02 00 00 05 48 71
Return: 01 06 02 00 00 05 48 71
Description: Change the address 0x01 to 0x05.
5. Modify the baud rate:
Command: 05 06 02 01 00 01 19 F6
Return: 05 06 02 01 00 01 19 F6
Description: Slave address is 0x05, change the baud rate to 0x01 (2400bps)
*/

unsigned char cmd[] = {0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0x85, 0xf6}; // 温度補正あり
// unsigned char cmd[] = {0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xd4, 0x36}; // 温度補正無し
float altitude = 2.0; // 高度(m)

void altitude_task(void *pvParameters)
{
  uint8_t buff[16];
  while (1)
  {
    ALTSerial.write(cmd, sizeof(cmd));

    delay(50);

    if (ALTSerial.available() >= 7)
    {
      ALTSerial.readBytes(buff, 7);

      uint16_t dist = (buff[3] << 8) | buff[4];
      altitude = dist / 1000.0f;
    }

    while (ALTSerial.available() > 0)
      ALTSerial.read();

    delay(50);
  }
}

void GetAltitude()
{
}

void InitAltitude()
{
  ALTSerial.begin(9600, SWSERIAL_8N1, ALT_RX, ALT_TX);
  xTaskCreate(altitude_task, "altitude_task", 2048, NULL, 1, NULL);
}
#pragma endregion

#pragma region TACHO
// 対気速度計のコード
hw_timer_t *timer = NULL;

volatile uint16_t tach_interrupts = 0;

boolean slit_rori_last = true;

uint32_t tach_rotation = 0;
uint32_t tach_last_time = 0;
uint32_t tach_delta_time = 0;

float air_speed = 30.0;

const uint32_t min_tach_delta = 150000;

void GetTacho()
{
  tach_delta_time = micros() - tach_last_time;

  if (tach_delta_time > min_tach_delta)
  {
    noInterrupts();
    if (tach_interrupts > 5)
    {
      tach_rotation = (uint32_t)((double)1000000000.0 * ((double)tach_interrupts / tach_delta_time));
      air_speed = (tach_rotation * tach_rotation * -7.0 * pow(10, -16.0) + tach_rotation * 3.0 * pow(10, -7.0)) * 1.0;
    }
    else
    {
      tach_rotation = 0;
      air_speed = 0;
    }
    interrupts();
    tach_interrupts = 0;
    tach_last_time = micros();
  }
}

void IRAM_ATTR tach_interrupt_count()
{
  if (digitalRead(TACHO_PIN[0]) != slit_rori_last)
  {
    tach_interrupts++;
    slit_rori_last = !slit_rori_last;
  }
}

void InitTacho()
{
  pinMode(TACHO_PIN[0], INPUT);
  pinMode(TACHO_PIN[1], INPUT);

  interrupts();
  timer = timerBegin(0, getApbFrequency() / 1000000, true);
  timerAttachInterrupt(timer, &tach_interrupt_count, true);
  timerAlarmWrite(timer, 20, true);
  timerAlarmEnable(timer);
  tach_last_time = micros();
}
#pragma endregion

#pragma region ROTATION_SPEED
// 回転数計のコード
const int SLIT_NUM = 36;
volatile uint16_t propeller_interrupts = 0;
uint32_t propeller_rotation = 106666;
uint32_t propeller_last_time = 0;
#define min_interrupts 5

uint32_t propeller_delta_time = 0;

const uint32_t min_propeller_delta = 500000;

void propeller_interrupt_count()
{
  propeller_interrupts++;
}

void attachPropeller()
{
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), propeller_interrupt_count, CHANGE);
}

void detachPropeller()
{
  detachInterrupt(digitalPinToInterrupt(RPM_PIN));
}

void GetRPM()
{
  propeller_delta_time = micros() - propeller_last_time;
  if (propeller_delta_time > min_propeller_delta)
  {
    detachPropeller(); // 計算中に値が変わらないようにする
    if (propeller_interrupts > min_interrupts)
    {
      // propeller_rotation = 割り込み数 / (スリット数 * 2.0)
      propeller_rotation = propeller_interrupts / (2.0 * SLIT_NUM) / (propeller_delta_time / 1000000.0) * 60.0;
    }
    else
    {
      propeller_rotation = 0;
    }
    attachPropeller();
    propeller_interrupts = 0;
    propeller_last_time = micros();
  }
}

void InitRPM()
{
  pinMode(RPM_PIN, INPUT_PULLUP);

  attachPropeller();
  interrupts();
  propeller_last_time = micros();
}
#pragma endregion

#pragma region LOG
// ログをmicroSDに保存
File fp;
constexpr char FILE_PATH[] = "/data.csv";
#define PRINT_COMMA fp.print(", ")

void InitSD();

void SDWriteTask(void *pvParameters)
{
  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);

  if (!SD.begin(SD_SPI_CS_PIN, SPI, 25000000))
  {
    delay(5000);
    InitSD();
    vTaskDelete(NULL);
    return;
  }

  if (!SD.exists(FILE_PATH))
  {
    fp = SD.open(FILE_PATH, FILE_WRITE);

    // 基準値の設定
    fp.println(
        "Date, Time, Latitude, Longitude, GPSAltitude, GPSCourse, GPSSpeed, "
        "AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, "
        "Roll_Mad6, Pitch_Mad6, Yaw_Mad6, Roll_Mad9, Pitch_Mad9, Yaw_Mad9, "
        "Roll_Mah6, Pitch_Mah6, Yaw_Mah6, Roll_Mah9, Pitch_Mah9, Yaw_Mah9, "
        "Temperature, Pressure, GroundPressure, BMPAltitude, Altitude, AirSpeed, "
        "PropellerRotationSpeed, Rudder, Elevator, RunningTime");
  }
  else
  {
    fp = SD.open(FILE_PATH, FILE_APPEND);
  }

  uint32_t start_time = millis();

  while (fp)
  {
    fp.printf("%d/%d/%d", gps_year, gps_month, gps_day);
    PRINT_COMMA;
    fp.printf("%d:%02d:%02d", gps_hour, gps_minute, gps_second);
    PRINT_COMMA;
    fp.printf("%.9f", gps_latitude);
    PRINT_COMMA;
    fp.printf("%.9f", gps_longitude);
    PRINT_COMMA;
    fp.print(gps_altitude);
    PRINT_COMMA;
    fp.print(gps_course);
    PRINT_COMMA;
    fp.print(gps_speed);
    PRINT_COMMA;
    fp.print(accelX);
    PRINT_COMMA;
    fp.print(accelY);
    PRINT_COMMA;
    fp.print(accelZ);
    PRINT_COMMA;
    fp.print(gyroX);
    PRINT_COMMA;
    fp.print(gyroY);
    PRINT_COMMA;
    fp.print(gyroZ);
    PRINT_COMMA;
    fp.print(magX);
    PRINT_COMMA;
    fp.print(magY);
    PRINT_COMMA;
    fp.print(magZ);
    PRINT_COMMA;
    fp.print(roll_mad6);
    PRINT_COMMA;
    fp.print(pitch_mad6);
    PRINT_COMMA;
    fp.print(yaw_mad6);
    PRINT_COMMA;
    fp.print(roll_mad9);
    PRINT_COMMA;
    fp.print(pitch_mad9);
    PRINT_COMMA;
    fp.print(yaw_mad9);
    PRINT_COMMA;
    fp.print(roll_mah6);
    PRINT_COMMA;
    fp.print(pitch_mah6);
    PRINT_COMMA;
    fp.print(yaw_mah6);
    PRINT_COMMA;
    fp.print(roll_mah9);
    PRINT_COMMA;
    fp.print(pitch_mah9);
    PRINT_COMMA;
    fp.print(yaw_mah9);
    PRINT_COMMA;
    fp.print(temperature);
    PRINT_COMMA;
    fp.print(pressure);
    PRINT_COMMA;
    fp.print(ground_pressure);
    PRINT_COMMA;
    fp.print(bmp_altitude);
    PRINT_COMMA;
    fp.print(altitude);
    PRINT_COMMA;
    fp.print(air_speed);
    PRINT_COMMA;
    fp.print(propeller_rotation);
    PRINT_COMMA;
    fp.printf("%.3f", rudder_rotation);
    PRINT_COMMA;
    fp.printf("%.3f", elevator_rotation);
    PRINT_COMMA;
    fp.print(millis() / 1000.0);
    PRINT_COMMA;
    fp.println();
    delay(50);

    if (millis() - start_time > 10000)
    {
      // 10秒ごとにファイルを閉じて再オープン
      break;
    }
  }

  fp.close();
  SD.end();
  InitSD();
  vTaskDelete(NULL);
}

void InitSD()
{
  xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 4096, NULL, 1, NULL, 0);
}
#pragma endregion

#pragma region SERVER
// HTTPサーバーでの処理
WebServer server(80);

void handleRoot()
{
  server.send(HTTP_CODE_OK, "text/plain", "index");
}
void handleNotFound()
{
  server.send(HTTP_CODE_NOT_FOUND, "text/plain", "Not Found");
}
void handleSetGroundPressure()
{
  if (server.hasArg("Pressure"))
  {
    ground_pressure = server.arg("Pressure").toFloat();
  }
  server.send(HTTP_CODE_OK, "text/plain", String(ground_pressure));
}
void handleGetGroundPressure()
{
  server.send(HTTP_CODE_OK, "text/plain", String(ground_pressure));
}
void handleSetServoRotation()
{
  // 操舵系統から値を受信する。サーボが動くわけではないので注意
  if (server.hasArg("Rudder"))
  {
    rudder_rotation = server.arg("Rudder").toFloat();
  }
  if (server.hasArg("Elevator"))
  {
    elevator_rotation = server.arg("Elevator").toFloat();
  }
  String str;
  str += rudder_rotation;
  str += ", ";
  str += elevator_rotation;
  server.send(HTTP_CODE_OK, "text/plain", str);
}
void handleGetMeasurementData()
{
  // JSONを作成する
  JsonDocument json_array;
  char json_string[4096];
  // JSONに変換したいデータを連想配列で指定する
  json_array["Year"] = gps_year;
  json_array["Month"] = gps_month;
  json_array["Day"] = gps_day;
  json_array["Hour"] = gps_hour;
  json_array["Minute"] = gps_minute;
  json_array["Second"] = gps_second;
  json_array["Latitude"] = gps_latitude;
  json_array["Longitude"] = gps_longitude;
  json_array["GPSAltitude"] = gps_altitude;
  json_array["GPSCourse"] = gps_course;
  json_array["GPSSpeed"] = gps_speed;
  json_array["AccelX"] = accelX;
  json_array["AccelY"] = accelY;
  json_array["AccelZ"] = accelZ;
  json_array["GyroX"] = gyroX;
  json_array["GyroY"] = gyroY;
  json_array["GyroZ"] = gyroZ;
  json_array["MagX"] = magX;
  json_array["MagY"] = magY;
  json_array["MagZ"] = magZ;
  json_array["Roll_Mad6"] = roll_mad6;
  json_array["Pitch_Mad6"] = pitch_mad6;
  json_array["Yaw_Mad6"] = yaw_mad6;
  json_array["Roll_Mad9"] = roll_mad9;
  json_array["Pitch_Mad9"] = pitch_mad9;
  json_array["Yaw_Mad9"] = yaw_mad9;
  json_array["Roll_Mah6"] = roll_mah6;
  json_array["Pitch_Mah6"] = pitch_mah6;
  json_array["Yaw_Mah6"] = yaw_mah6;
  json_array["Roll_Mah9"] = roll_mah9;
  json_array["Pitch_Mah9"] = pitch_mah9;
  json_array["Yaw_Mah9"] = yaw_mah9;
  json_array["Temperature"] = temperature;
  json_array["Pressure"] = pressure;
  json_array["GroundPressure"] = ground_pressure;
  json_array["BMPAltitude"] = bmp_altitude;
  json_array["Altitude"] = altitude;
  json_array["AirSpeed"] = air_speed;
  json_array["PropellerRotationSpeed"] = propeller_rotation;
  json_array["Rudder"] = rudder_rotation;
  json_array["Elevator"] = elevator_rotation;
  json_array["RunningTime"] = millis() / 1000.0;

  // JSONフォーマットの文字列に変換する
  serializeJson(json_array, json_string, sizeof(json_string));

  server.send(HTTP_CODE_OK, "text/plain", json_string);
}

void InitServer()
{
  setupOTA("FM5", SSID, PASSPHRASE);
  server.on("/", handleRoot);
  server.on("/SetGroundPressure", handleSetGroundPressure);
  server.on("/GetGroundPressure", handleGetGroundPressure);
  server.on("/SetServoRotation", handleSetServoRotation);
  server.on("/GetMeasurementData", handleGetMeasurementData);
  server.onNotFound(handleNotFound);
  server.begin();
}
#pragma endregion

void setup()
{
  Serial.begin(115200);

  auto cfg = M5.config();
  CoreS3.begin(cfg);
  CoreS3.Ex_I2C.begin();

  InitBMP280();
  delay(100);
  InitBMI270();
  delay(100);
  InitGPS();
  delay(100);
  InitTacho();
  delay(100);
  InitRPM();
  delay(100);

  InitSD();
  delay(100);
  InitServer();
  delay(100);
}

void loop()
{
  CoreS3.update();

  if (WiFi.status() == WL_CONNECTED)
  {
    server.handleClient();
  }
  GetBMP280();
  GetBMI270();
  GetGPS();
  GetAltitude();
  GetTacho();
  GetRPM();

  // Display
  static uint32_t last_print = 0;
  if (millis() - last_print > 500)
  {
    last_print = millis();

    CoreS3.Display.setCursor(0, 0);
    CoreS3.Display.clear();

    // CoreS3.Display.printf("Temperature: %f *C\r\n", temperature);
    // CoreS3.Display.printf("Pressure: %f Pa\r\n", pressure);
    CoreS3.Display.printf("Altitude: %f m\r\n", altitude);
    // CoreS3.Display.printf("BMP Altitude: %f m\r\n", bmp_altitude);
    CoreS3.Display.printf("GPS Time: %d/%d/%d %d:%02d:%02d\r\n", gps_year, gps_month, gps_day, gps_hour, gps_minute, gps_second);
    CoreS3.Display.printf("Latitude: %.9f, Longitude: %.9f\r\n", gps_latitude, gps_longitude);
    // CoreS3.Display.printf("GPS Altitude: %f m\r\n", gps_altitude);
    // CoreS3.Display.printf("GPS Course: %f deg\r\n", gps_course);
    // CoreS3.Display.printf("GPS Speed: %f m/s\r\n", gps_speed);
    // CoreS3.Display.printf("ax: %f, ay: %f, az: %f\r\n", accelX, accelY, accelZ);
    // CoreS3.Display.printf("gx: %f, gy: %f, gz: %f\r\n", gyroX, gyroY, gyroZ);
    // CoreS3.Display.printf("mx: %d, my: %d, mz: %d\r\n", magX, magY, magZ);
    CoreS3.Display.printf("Roll: %f, Pitch: %f, Yaw: %f\r\n", roll_mad9, pitch_mad9, yaw_mad9);
    CoreS3.Display.printf("Air Speed: %f\r\n", air_speed);
    CoreS3.Display.printf("Propeller Rotation Speed: %d\r\n", propeller_rotation);
    CoreS3.Display.printf("Rudder: %f, Elevator: %f\r\n", rudder_rotation, elevator_rotation);
  }
}
