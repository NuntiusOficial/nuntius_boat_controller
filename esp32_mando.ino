#include <EEPROM.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <time.h>

#include "RF24.h"
#include "RF24_config.h"
#include "nRF24L01.h"
#include "src/ArduinoJson/ArduinoJson.h"

#define CIRCULAR_BUFFER_SIZE 32

QueueHandle_t queueDataReceived;
void bleSendTask(void *pvParameters);
void bleReceiveTask(void *pvParameters);
void espNowSendTask(void *pvParameters);
HardwareSerial ble(1);

// Peer info
esp_now_peer_info_t peerInfo;
const uint8_t broadcastAddress[] = {0x94, 0xb9, 0x7e, 0xce, 0x05, 0x94}; //C8:F0:9E:A6:58:34    94:b9:7e:ce:05:94
volatile unsigned long success = 0, fail = 0;
volatile float dataRate = 0;
volatile bool espNowError;

//******************************************************************************
// variables globales
char btData[100];
bool newData = false;
uint8_t calibStatus = 0;
bool BtHasConnected = false;
bool autoPilot = false;
bool autoHome = false;
bool motorCorrectionTest = false;
uint8_t motorCorrectionSpeed = 0;
String wifiData;
char ssid[20];
char psw[20];
const char *currentVersion = "0.0.0";

bool LuzIntermitente = false;
uint8_t timerTolba = 0;
uint8_t refrescoSerie = 30;
uint8_t Power360 = 10;
uint8_t CruiseSpeed = 30;
float CruiseSpeedError = 0;
bool Autocruise = false;
bool SaveEnergy = false;
uint8_t Comunicando = 0;
uint8_t csped = 0;
uint8_t BateriaLocal1 = 0;
uint8_t BateriaLocal2 = 0;
uint8_t BateriaBarco = 0;
uint8_t Divisor = 0;
uint8_t PwMaxMotores = 70;
int16_t CurVel = 0;
int8_t CurDir = 0;
bool signalB = false;  // se#al comunicacion barco mando
bool BtError = false;
int Xdeviation;
unsigned long OkTime;
int g1 = 512, g2 = 512;  // joystick origin
float g = 0;
volatile bool stillOnCb;
bool autoCorr;
bool boatInit;
bool updateSent = false;
enum Command {
    NOTHING,
    SAVE_HOME,
    AUTOPILOT,
    GO_HOME,
    STOP,
    COMPASS_CAL,
    COMPASS_CAL_AUTO,
    ACCEL_CAL,
    AUTO_HOME,
    AUTO_TOLVA,
    WIFI_REPEATER,
    MAX_SPEED,
    ANGLE_CORRECTION,
    TEMP_CORRECTION,
    PRESS_CORRECTION,
    TOLVA_L_CORRECTION,
    TOLVA_R_CORRECTION,
    MOTOR_L_INVERSION,
    MOTOR_R_INVERSION,
    INIT_ANGLE_CORRECTION,
    AUTOPILOT_COMPENSATION,
    MOTOR_CORRECTION_10,
    MOTOR_CORRECTION_20,
    MOTOR_CORRECTION_30,
    MOTOR_CORRECTION_40,
    MOTOR_CORRECTION_50,
    MOTOR_CORRECTION_60,
    MOTOR_CORRECTION_70,
    MOTOR_CORRECTION_80,
    MOTOR_CORRECTION_90,
    MOTOR_CORRECTION_100,
    AUTO_MOTOR_CORRECTION,
    RESET,
    UPDATE,
} command;

enum class Info {
    NOTHING,
    AUTO_CORR_COMPLETED,
    BOAT_INITIATED,
} info;

//######################## Data Structures   #######################

struct CoordGPS {
    float lat = 0.0;
    float lon = 0.0;
};
CoordGPS Home;
CoordGPS Destino;

struct Data {
    uint8_t Power360;
    uint8_t CruiseSpeed;
    float CruiseSpeedError;
    uint8_t PwMaxMotores;
    uint8_t csped;
    float lat;
    float lon;
} dataSaved;

struct RadioTXPacket {
    uint8_t Luces = 0;           // 0=apagada, 1=Encendido min, 2=Encendido max, 3=intermitente min, 4=intermitente max
    int8_t MotorR = 0;           // +100 max adelante, -100 max atrás
    int8_t MotorL = 0;           // +100 max adelante, -100 max atrás
    uint8_t tolba;               // Spot 360
    uint8_t tolba1 = 90;         // tolba 1
    uint8_t tolba2 = 90;         // tolba 2
    uint8_t Luzcamara;           // '0'=apagada, '1'=Encendido
    uint8_t Comando = 0;         // 0=nada, 1=Posicion Home, 2=Navegar a lat lon, 3=Navegar a Home
    float lat = 1.0;             // destino GPS
    float lon = 0.0;             // destino GPS
    uint8_t CamaraInferior = 0;  // '0' quieta, '1' sube, '2' baja
    uint8_t Camara360;           // de 0 a 36, siendo 0... 0grados y 36 360grados
    uint8_t CamaraActiva;        // '1' la superior '0' la de profundidad
    bool tolvProg;               // '1' la superior '0' la de profundidad
    float CruiseSpeedError;
    float commandValue;
    bool BtError;
    uint8_t canal;
    char txtData[64];
} mandoTX;

struct RadioRXPacket {
  uint8_t batt1;  // 0...100 lectura directa a procesar por el mando
  uint8_t batt2;  // 0...100 lectura directa a procesar por el mando
  uint8_t speed;
  uint8_t comando;
  uint8_t apState;
  uint8_t satelites;     // numero de satelites validos
  uint16_t presion;      // presion
  uint16_t temperatura;  // Temperatura
  uint16_t Altitud;      // Altitud
  uint16_t gyros;
  int16_t c1;
  int16_t c2;
  int16_t motorL;
  int16_t motorR;
  float lat;             // latitud GPS actual
  float lon;             // longitud GPS actual
  uint16_t info;
} __attribute__((packed)) barcoTX;

//################## Network Communication #########################

// Set time via NTP, as required for x.509 validation
void setClock() {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");  // UTC

    Serial.print(F("Waiting for NTP time sync: "));
    time_t now = time(nullptr);
    while (now < 8 * 3600 * 2) {
        yield();
        delay(500);
        Serial.print(F("."));
        now = time(nullptr);
    }

    Serial.println(F(""));
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    Serial.print(F("Current time: "));
    Serial.print(asctime(&timeinfo));
}

/**
   This is lets-encrypt-x3-cross-signed.pem
*/
const char *rootCACertificate =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIFFjCCAv6gAwIBAgIRAJErCErPDBinU/bWLiWnX1owDQYJKoZIhvcNAQELBQAw\n"
    "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
    "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjAwOTA0MDAwMDAw\n"
    "WhcNMjUwOTE1MTYwMDAwWjAyMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg\n"
    "RW5jcnlwdDELMAkGA1UEAxMCUjMwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK\n"
    "AoIBAQC7AhUozPaglNMPEuyNVZLD+ILxmaZ6QoinXSaqtSu5xUyxr45r+XXIo9cP\n"
    "R5QUVTVXjJ6oojkZ9YI8QqlObvU7wy7bjcCwXPNZOOftz2nwWgsbvsCUJCWH+jdx\n"
    "sxPnHKzhm+/b5DtFUkWWqcFTzjTIUu61ru2P3mBw4qVUq7ZtDpelQDRrK9O8Zutm\n"
    "NHz6a4uPVymZ+DAXXbpyb/uBxa3Shlg9F8fnCbvxK/eG3MHacV3URuPMrSXBiLxg\n"
    "Z3Vms/EY96Jc5lP/Ooi2R6X/ExjqmAl3P51T+c8B5fWmcBcUr2Ok/5mzk53cU6cG\n"
    "/kiFHaFpriV1uxPMUgP17VGhi9sVAgMBAAGjggEIMIIBBDAOBgNVHQ8BAf8EBAMC\n"
    "AYYwHQYDVR0lBBYwFAYIKwYBBQUHAwIGCCsGAQUFBwMBMBIGA1UdEwEB/wQIMAYB\n"
    "Af8CAQAwHQYDVR0OBBYEFBQusxe3WFbLrlAJQOYfr52LFMLGMB8GA1UdIwQYMBaA\n"
    "FHm0WeZ7tuXkAXOACIjIGlj26ZtuMDIGCCsGAQUFBwEBBCYwJDAiBggrBgEFBQcw\n"
    "AoYWaHR0cDovL3gxLmkubGVuY3Iub3JnLzAnBgNVHR8EIDAeMBygGqAYhhZodHRw\n"
    "Oi8veDEuYy5sZW5jci5vcmcvMCIGA1UdIAQbMBkwCAYGZ4EMAQIBMA0GCysGAQQB\n"
    "gt8TAQEBMA0GCSqGSIb3DQEBCwUAA4ICAQCFyk5HPqP3hUSFvNVneLKYY611TR6W\n"
    "PTNlclQtgaDqw+34IL9fzLdwALduO/ZelN7kIJ+m74uyA+eitRY8kc607TkC53wl\n"
    "ikfmZW4/RvTZ8M6UK+5UzhK8jCdLuMGYL6KvzXGRSgi3yLgjewQtCPkIVz6D2QQz\n"
    "CkcheAmCJ8MqyJu5zlzyZMjAvnnAT45tRAxekrsu94sQ4egdRCnbWSDtY7kh+BIm\n"
    "lJNXoB1lBMEKIq4QDUOXoRgffuDghje1WrG9ML+Hbisq/yFOGwXD9RiX8F6sw6W4\n"
    "avAuvDszue5L3sz85K+EC4Y/wFVDNvZo4TYXao6Z0f+lQKc0t8DQYzk1OXVu8rp2\n"
    "yJMC6alLbBfODALZvYH7n7do1AZls4I9d1P4jnkDrQoxB3UqQ9hVl3LEKQ73xF1O\n"
    "yK5GhDDX8oVfGKF5u+decIsH4YaTw7mP3GFxJSqv3+0lUFJoi5Lc5da149p90Ids\n"
    "hCExroL1+7mryIkXPeFM5TgO9r0rvZaBFOvV2z0gp35Z0+L4WPlbuEjN/lxPFin+\n"
    "HlUjr8gRsI3qfJOQFy/9rKIJR0Y/8Omwt/8oTWgy1mdeHmmjk7j1nYsvC9JSQ6Zv\n"
    "MldlTTKB3zhThV1+XWYp6rjd5JW1zbVWEkLNxE7GJThEUG3szgBVGP7pSWTUTsqX\n"
    "nLRbwHOoq7hHwg==\n"
    "-----END CERTIFICATE-----\n";


void sendCommand(const char *command) {
    Serial.print("Command send :");
    Serial.println(command);
    ble.println(command);
    // wait some time
    delay(100);

    char reply[100];
    int i = 0;
    while (ble.available()) {
        reply[i] = ble.read();
        i += 1;
    }
    // end the string
    reply[i] = '\0';
    Serial.print(reply);
    Serial.println("Reply successful");
    delay(50);
}

void setup() {
    ble.begin(9600, SERIAL_8N1, 16, 17);
    Serial.begin(115200);
    pinMode(32, OUTPUT);
    pinMode(33, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);

    sendCommand("AT");
    sendCommand("AT+ROLE0");
    sendCommand("AT+UUID0xFFE0");
    sendCommand("AT+CHAR0xFFE1");
    sendCommand("AT+NAMELightBlue");
    
    WiFi.mode(WIFI_STA);
    int a= esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_LR );
    Serial.println(a);

    // Initilize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register the send & receive callback
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    delay(100);

    queueDataReceived = xQueueCreate(1,sizeof(mandoTX));

    // Now set up two tasks to run independently.
    xTaskCreatePinnedToCore(
        bleSendTask, "bleSendTask"  // A name just for humans
        ,
        1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        (void *)&barcoTX, 2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
        espNowSendTask, "espNowSendTask"  // A name just for humans
        ,
        4092  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        (void *)&mandoTX, 3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
        bleReceiveTask, "bleReceiveTask", 2048  // Stack size
        ,
        (void *)&btData, 2  // Priority
        ,
        NULL, ARDUINO_RUNNING_CORE);

    // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
    Serial.println("The device started, now you can pair it with bluetooth!");
    eepromRead();
    Serial.println(sizeof(barcoTX));
    // Set ESP32 as a Wi-Fi Station

}

void loop() {
    static unsigned long count;
    static float distanceHome;
    static int errorCount = 0;
    static bool comError = false;
    static unsigned long prev_time_signal = 0;
    static unsigned long prev_time_batt = 0;
    int batt1_porcent, batt2_porcent;
    static uint8_t transmitterCount = 0;
    static uint8_t successTransmit = 0;
    static uint8_t transmitionRate = 100;
    static bool hasConnected = false;
    static unsigned long last_data_rate = millis();
    if ( newData ) serial_1_Com(btData);
    //Serial.println(btData);
    // ----- Tareas cada 10mseg -----------------------------------------------------------------------------
    if (Ticker()) {   
        LeerVelocidad();
        mandoTX.CruiseSpeedError = CruiseSpeedError;

    }
    //------------------------------------------------------------------------------------------------------------

    // Hay paquete en la radio? -----------------
    /*while (radio.available()) {

      Comunicando = 80;
      radio.read(&barcoTX, sizeof(barcoTX));
      if (barcoTX.comando == mandoTX.Comando) {
        if (barcoTX.comando == 45) { //comando para cambio de frecuencia
          radio.setChannel(mandoTX.canal);
          hasConnected = false;
          Serial.println(F("Cambiando de canal"));
        }


        if (barcoTX.comando == 7) calibStatus = 2;
        else if (barcoTX.comando == 6) calibStatus = 4;
        else if (barcoTX.comando == 5) calibStatus = 6;
        mandoTX.Comando = NOTHING;
      }
      RefrescarDisplay();
      }*/

    if (((millis() - OkTime) > 3600000) && BtHasConnected)
        BtError = true;
    else
        BtError = false;

    mandoTX.BtError = BtError;

    if (!signalB || BtError) {
        if ((millis() - prev_time_signal) > 500) {
            prev_time_signal = millis();
        }
    }

    if (batt1_porcent < 15 || batt2_porcent < 15) {
        if ((millis() - prev_time_batt) > 500) {
            prev_time_batt = millis();
        }
    }
}



void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    // if (alreadyPaired == false) alreadyPaired = true;
    // char msg[27];
    //Serial.println("Bytes received: " + String(len));
    //Serial.println(sizeof(barcoTX));
    memcpy(&barcoTX, incomingData, len);
        // Serial.println("data recevied from espnow");
        // Serial.print("lat: ");
        // Serial.println(barcoTX.lat, 6);
        // Serial.print("lon: ");
        // Serial.println(barcoTX.lon, 6);
        Serial.print("bat1: ");
        Serial.println(barcoTX.batt1);
        Serial.print("bat2: ");
        Serial.println(barcoTX.batt2);
        // Serial.print("gyro: ");
        // Serial.println(barcoTX.gyros);
        Serial.print("temp: ");
        Serial.println(barcoTX.temperatura);
        // Serial.print("pressure: ");
        // Serial.println(barcoTX.presion);

        if (barcoTX.info == (uint16_t) Info::AUTO_CORR_COMPLETED) {
            autoCorr = true;
        }

        if (barcoTX.info == (uint16_t) Info::BOAT_INITIATED) {
            boatInit = true;
        }

    if (barcoTX.comando == mandoTX.Comando) {
        /*if (barcoTX.comando == 45) { //comando para cambio de frecuencia
          radio.setChannel(mandoTX.canal);
          hasConnected = false;
          Serial.println(F("Cambiando de canal"));
        }*/

        if (barcoTX.comando == 7) calibStatus = 2;
        else if (barcoTX.comando == 6) calibStatus = 4;
        else if (barcoTX.comando == 5) calibStatus = 6;
        mandoTX.Comando = NOTHING;
      }

}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (!stillOnCb){
        stillOnCb = true;
        if (status == ESP_NOW_SEND_SUCCESS) {
            success++;
            if(mandoTX.Comando == UPDATE){
                mandoTX.Comando = NOTHING;
                updateSent =  true;
            }
            updateDataRate(1);
        } else {
            fail++;
            updateDataRate(0);
            Serial.println("failed");
            Serial.println(esp_err_to_name(status));
        }
        stillOnCb = false;
    }else{
        Serial.println("not finnished callback");
    }
}

void checkUpdate() {
    for (uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] WAIT %d...\n", t);
        Serial.flush();
        delay(1000);
    }
    int a= esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
    Serial.println(a);
    delay(100);

    Serial.println(ssid);
    Serial.println(psw);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, psw);
    delay(3000);
    while ((WiFi.status() != WL_CONNECTED)) {
        WiFi.begin(ssid, psw);
        delay(5000);
    };

    Serial.println("Connected");
    setClock();

    Serial.println(ESP.getFreeHeap());
    delay(1000);

    WiFiClientSecure client;
  
    Serial.println(ESP.getFreeHeap());
    delay(1000);

    client.setCACert(rootCACertificate);
  
    Serial.println(ESP.getFreeHeap());
    delay(1000);

    // Reading data over SSL may be slow, use an adequate timeout
    client.setTimeout(12000 / 1000);  // timeout argument is defined in seconds for setTimeout

    // The line below is optional. It can be used to blink the LED on the board during flashing
    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
    // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
    // httpUpdate.setLedPin(LED_BUILTIN, HIGH);

    String updateFile = getUpdateJson();

    Serial.println("asking for json");
    delay(1000);

    DynamicJsonDocument doc(255);
    // Deserialize the JSON document in the response
    deserializeJson(doc, updateFile);

    if (doc["Version"].as<String>().equalsIgnoreCase(currentVersion)) {
        Serial.println("Already Updated");
        ESP.restart();
        return;
    }

    t_httpUpdate_return ret = httpUpdate.update(client, doc["Path"].as<String>());
    // Or:
    // t_httpUpdate_return ret = httpUpdate.update(client, "server", 443, "/file.bin");

    switch (ret) {
        case HTTP_UPDATE_FAILED:
            Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
            // SerialBT.begin();
            break;

        case HTTP_UPDATE_NO_UPDATES:
            Serial.println("HTTP_UPDATE_NO_UPDATES");
            // SerialBT.begin();
            break;

        case HTTP_UPDATE_OK:
            // Serial.println("HTTP_UPDATE_OK");
            break;
    }
}

String getUpdateJson() {
    HTTPClient http;
    uint8_t retry = 0;
    int httpResponseCode;
    
    Serial.println("inside getjsonUpdate");
    Serial.println(ESP.getFreeHeap());
    delay(1000);

    ESP_LOGI("getUpdateJson", "HIGHWATER: %d", uxTaskGetStackHighWaterMark(NULL));

    if (WiFi.status() == WL_CONNECTED) {  // Check WiFi connection status
        do {
            http.begin("https://www.nuntius.club/remote_controller_stable_release.json");  // Specify destination for HTTP request
            http.addHeader("Content-Type", "application/json");  
            Serial.println("after http Begin");
            Serial.println(ESP.getFreeHeap());
            ESP_LOGI("getUpdateJson", "HIGHWATER: %d", uxTaskGetStackHighWaterMark(NULL));
            delay(1000);

            int httpResponseCode = http.GET();

            
            Serial.println("after http get");
            Serial.println(ESP.getFreeHeap());
            delay(1000);

            String response = http.getString();  // Get the response to the request

            Serial.println("after http getString");
            Serial.println(ESP.getFreeHeap());
            delay(1000);

            Serial.println(httpResponseCode);  // Print return code
            Serial.println(response);          // Print request answer

            http.end();  // Free resources

            if (httpResponseCode == 200) {
                return response;
            }

            delay(500);  // wait 500ms before retry.circular
            retry++;
            Serial.println("retry " + String(retry) + "/" + String(5));
        } while (retry < 5);

    } else {
        Serial.println("Error in WiFi connection");
    }
    return "";
}

void eepromRead() {
    EEPROM.begin(sizeof(dataSaved));
    EEPROM.get(0,dataSaved);

    dataSaved.PwMaxMotores = (isnan(dataSaved.PwMaxMotores) || dataSaved.PwMaxMotores == 0)? dataSaved.PwMaxMotores : 100;
    dataSaved.CruiseSpeed = (isnan(dataSaved.CruiseSpeed) || dataSaved.CruiseSpeed == 0)? dataSaved.CruiseSpeed : 50;

    PwMaxMotores = dataSaved.PwMaxMotores;  // valor guradado en memoria
    CruiseSpeedError = dataSaved.CruiseSpeedError;
    CruiseSpeed = dataSaved.CruiseSpeed;
    Power360 = dataSaved.Power360;
    csped = dataSaved.csped;
    Home.lat = dataSaved.lat;
    Home.lon = dataSaved.lon;

    Serial.println(dataSaved.lat, 6);
    Serial.println(PwMaxMotores);
    Serial.println(CruiseSpeedError);
    Serial.println(CruiseSpeed);
    Serial.println(Power360);
    Serial.println(csped);
}

//******************************************************************************
uint32_t MiTicker = 100000UL;
uint32_t timeNowmseg = micros();

boolean Ticker(void) {
    unsigned long tmp;
    tmp = micros();
    if ((tmp - timeNowmseg) >= MiTicker) {
        timeNowmseg = tmp;
        if (timerTolba) {
            timerTolba--;
            if (!timerTolba) {
                mandoTX.tolba = 0;
                //        Display.sendCommand("q8.picc=1");
            }
        }
        return 1;
    }
    return 0;
}

//******************************************************************************
static int16_t Vel = 0;
static int8_t Dir = 0;
int8_t MaxVal;
int curVel, curDir;
int cont = 0;
void LeerVelocidad(void) {
    int decreasing_factor = 3;

    if (motorCorrectionTest) {
        mandoTX.MotorR = motorCorrectionSpeed;
        mandoTX.MotorL = motorCorrectionSpeed;
    } else if (Autocruise == true) {
        if (SaveEnergy) {
            Vel = CruiseSpeed / 2;
        }

        else
            Vel = CruiseSpeed;

        mandoTX.MotorR = Vel;
        mandoTX.MotorL = Vel;
        Dir = cont;

        int deviation = Xdeviation;

        if (deviation < 10 && deviation > -10) {
            deviation = 0;
        }

        deviation = map(deviation, -100, 100, -25, 25);
        // Serial.println("Desviación: "+String(deviation));
        if (deviation < 0) {
            mandoTX.MotorL += deviation;
        } else {
            mandoTX.MotorR -= deviation;
        }

        if (mandoTX.MotorR > Vel) {
            mandoTX.MotorR = Vel;
        }
        if (mandoTX.MotorL > Vel) {
            mandoTX.MotorL = Vel;
        }

    } else {
        if (SaveEnergy == true)
            MaxVal = PwMaxMotores / 2;
        else
            MaxVal = PwMaxMotores;

        CurVel = map(g1, 0, 1023, -MaxVal, +MaxVal);
        CurDir = map(g2, 0, 1023, -MaxVal, +MaxVal);
        mandoTX.MotorR = CurVel;
        mandoTX.MotorL = CurDir;
    }
}

//******************************************************************************

void RefrescarDisplay(void) {
    static int calibCount;
    String tmp;
    static uint8_t apstate;
    static bool buttonAppState;
    static unsigned long prevTimeState;
    const unsigned long intervalTime = 500;
    if (--refrescoSerie) return;
    refrescoSerie = 10;

    apstate = barcoTX.apState;

    if ((apstate & 0b00001100) == 0b00000100) {  // if autoHome idle
        apstate = buttonAppState ? (apstate | 0b00001100) & 0b11110111 : apstate & 0b11110011;
    }

    if ((apstate & 0b00000011) == 0b00000001) {  // if autoTolva idle
        apstate = buttonAppState ? (apstate | 0b00000011) & 0b11111101 : apstate & 0b11111100;
    }

    if (millis() - prevTimeState > intervalTime) {
        buttonAppState = !buttonAppState;
        prevTimeState = millis();
    }

    String data = "{\"t\":";
    data += barcoTX.temperatura / 100.0;

    data = ",\"p\":";
    data += barcoTX.presion;
    data += ".0";
    // SerialBT.println(data);
    delay(10);

    data = ",\"b1\":";
    data += barcoTX.batt2;
    // SerialBT.println(data);
    delay(10);

    data = ",\"sg\":";
    data += signalB;
    // SerialBT.println(data);
    delay(10);

    data = ",\"b\":";
    data += barcoTX.batt1;
    // SerialBT.println(data);
    delay(10);

    data = ",\"lat\":";
    data += String(barcoTX.lat, 8);
    // SerialBT.println(data);
    delay(10);

    data = ",\"long\":";
    data += String(barcoTX.lon, 8);
    // SerialBT.println(data);
    delay(10);

    data = ",\"sat\":";
    data += barcoTX.satelites;
    // SerialBT.println(data);
    delay(10);

    data = ",\"gyro\":";
    data += barcoTX.gyros;
    // SerialBT.println(data);
    delay(10);

    // data += ",\"alertTolv\":false";
    data = ",\"speed\":";
    data += (float)barcoTX.speed / 10;
    // SerialBT.println(data);
    delay(10);

    data = ",\"c1\":";
    data += (float)barcoTX.c1 / 1000;
    // SerialBT.println(data);
    delay(10);

    data = ",\"c2\":";
    data += (float)barcoTX.c2 / 1000;
    // SerialBT.println(data);
    delay(10);

    data = ",\"calib\":";
    data += calibStatus;
    // SerialBT.println(data);
    delay(10);

    data = ",\"apState\":";
    data += apstate;
    // SerialBT.println(data);
    delay(10);

    data = ",\"motorl\":";
    data += barcoTX.motorL;
    // SerialBT.println(data);
    delay(10);

    data += ",\"motorr\":";
    data += barcoTX.motorR;
    data += "}";
    // SerialBT.println(data);
    delay(10);

    Serial.println(data);
    Serial.println("Command: " + String(mandoTX.Comando));

    if (calibStatus != 0) {
        calibCount++;
        if (calibCount > 3) {
            calibStatus = 0;
            calibCount = 0;
        }
    }
}

//******************************************************************************/

void serial_1_Com(char* data) {
    newData = false;
    String data1 =  String(data);
    Serial.println(data1);
    if (data1.length() > 0) {
        //      case '0': // Comando
        //        Serial.println(F("Minimo COM 1.0"));
        //        break;
        //
        //      case 'P': //
        //        PaginaDisplay = Char2Hex(RX.texto[0]);
        //        RefrescarDisplay();
        //        break;

        if (data1.indexOf("llegTolb:0") != -1) {  // Autotolvas off
            mandoTX.Comando = AUTO_TOLVA;
            mandoTX.commandValue = false;
            return;
        }
        if (data1.indexOf("upd") != -1) {  // Autotolvas off
            strcpy(mandoTX.txtData, wifiData.c_str());
            Serial.println(mandoTX.txtData);
            mandoTX.Comando = UPDATE;
            return;
        }

        if (data1.indexOf("ssid") != -1) {  // Autotolvas off
            wifiData = data1.substring(data1.indexOf(":") + 1);
            return;
        }

        if (data1.indexOf("psw") != -1) {  // Autotolvas off
            wifiData.concat(',');
            wifiData.concat(data1.substring(data1.indexOf(":") + 1));
            return;
        }

        if (data1.indexOf("autocorr:true") != -1) {  // Autotolvas off
            mandoTX.Comando = AUTO_MOTOR_CORRECTION;
            mandoTX.commandValue = true;
            return;
        }

        if (data1.indexOf("autocorr:false") != -1) {  // Autotolvas off
            mandoTX.Comando = AUTO_MOTOR_CORRECTION;
            mandoTX.commandValue = false;
            return;
        }

        if (data1.indexOf("reset") != -1) {  // Autotolvas off
            mandoTX.Comando = RESET;
            mandoTX.commandValue = false;
            return;
        }

        if (data1.indexOf("llegTolb:1") != -1) {  // autotolvas on
            mandoTX.Comando = AUTO_TOLVA;
            mandoTX.commandValue = true;
            return;
        }

        if (data1.indexOf("L1") != -1) {
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Luces = v;
        }

        // if (SaveEnergy) {
        // if (RX.texto[0] == '1')mandoTX.Luces = 1;
        // else mandoTX.Luces = 0;
        // }
        // else {
        // if (RX.texto[0] == '1')mandoTX.Luces = 2;
        // else mandoTX.Luces = 0;
        // }

        if (data1.indexOf("L2") != -1) {
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Luzcamara = v;
        }

        if (data1.indexOf("Rb") != -1) {
            int v = data1.substring(data1.indexOf(":") + 1).toInt();

            Autocruise = (v == 0) ? false : true;
        }
        if (data1.indexOf("vc") != -1) {
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            csped = CruiseSpeed = v;
            CruiseSpeed = map(CruiseSpeed, 0, 100, 0, PwMaxMotores);
            // Serial.println(CruiseSpeed);
            dataSaved.csped = csped;
            dataSaved.CruiseSpeed = CruiseSpeed;
            EEPROM.put(0, dataSaved);
        }
        if (data1.indexOf("vt") != -1) {
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            PwMaxMotores = v;
            csped = CruiseSpeed = PwMaxMotores;
            dataSaved.csped = csped;
            dataSaved.CruiseSpeed = CruiseSpeed;
            dataSaved.PwMaxMotores = PwMaxMotores;
            mandoTX.Comando = MAX_SPEED;
            mandoTX.commandValue = PwMaxMotores;
            EEPROM.put(0, dataSaved);
        }

        if (data1.indexOf("mc") != -1) {
            int state = data1.substring(data1.indexOf(":") + 1, data1.indexOf(",")).toInt();
            int value = data1.substring(data1.indexOf(",") + 1).toInt();
            motorCorrectionTest = state == 1 ? true : false;
            motorCorrectionSpeed = value;
        }

        if (data1.indexOf("sc") != -1) {
            int value = data1.substring(data1.indexOf(":") + 1).toInt();
            motorCorrectionSpeed = value;
        }

        if (data1.indexOf("mt") != -1) {
            int correctionIndex = data1.substring(data1.indexOf(":") + 1, data1.indexOf(",")).toInt();
            int correctionValue = data1.substring(data1.indexOf(",") + 1).toInt();

            CruiseSpeedError = correctionValue / 2;
            dataSaved.CruiseSpeedError = CruiseSpeedError;
            switch (correctionIndex) {
                case 1:
                    mandoTX.Comando = MOTOR_CORRECTION_10;
                    break;

                case 2:
                    mandoTX.Comando = MOTOR_CORRECTION_20;
                    break;

                case 3:
                    mandoTX.Comando = MOTOR_CORRECTION_30;
                    break;

                case 4:
                    mandoTX.Comando = MOTOR_CORRECTION_40;
                    break;

                case 5:
                    mandoTX.Comando = MOTOR_CORRECTION_50;
                    break;

                case 6:
                    mandoTX.Comando = MOTOR_CORRECTION_60;
                    break;

                case 7:
                    mandoTX.Comando = MOTOR_CORRECTION_70;
                    break;

                case 8:
                    mandoTX.Comando = MOTOR_CORRECTION_80;
                    break;

                case 9:
                    mandoTX.Comando = MOTOR_CORRECTION_90;
                    break;

                case 10:
                    mandoTX.Comando = MOTOR_CORRECTION_100;
                    break;

                default:
                    break;
            }
            mandoTX.CruiseSpeedError;
            EEPROM.put(0, dataSaved);
        }
        if (data1.indexOf("dt") != -1) {
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            Power360 = v;
            dataSaved.Power360 = Power360;
            EEPROM.put(0, dataSaved);
        }

        if (data1.indexOf("jc") != -1) {  // camara inferior
            Serial.println("cam1");
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.CamaraInferior = v;
            //       mandoTX.CamaraActiva = 0;
        }
        if (data1.indexOf("tolbPro:true") != -1) {
            mandoTX.tolvProg = 1;
        }
        if (data1.indexOf("tolbPro:false") != -1) {
            mandoTX.tolvProg = 0;
        }

        if (data1.indexOf("cam") != -1) {  // camara superio
            double g = 0, g1 = 0;
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Camara360 = v;
        }
        if (data1.indexOf("camsup") != -1) {
            mandoTX.CamaraActiva = 1;
        }
        if (data1.indexOf("caminf") != -1) {
            mandoTX.CamaraActiva = 0;
        }

        if (data1.indexOf("rr") != -1) {
            g1 = data1.substring(data1.indexOf(":") + 1, data1.indexOf(",")).toInt();
            g2 = data1.substring(data1.indexOf(",") + 1).toInt();

            // Valores Humbrales
            int humbral = 10;
            if (g1 < 50 + humbral && g1 > 50 - humbral)
                g1 = 50;

            if (g2 < 50 + humbral && g2 > 50 - humbral)
                g2 = 50;

            g1 = map(g1, 0, 100, -100, 100);
            g2 = map(g2, 0, 100, 100, -100);

            Xdeviation = g1;

            int cuadrante, new_g1, new_g2;

            if (g1 >= 0 && g2 >= 0) {
                cuadrante = 1;
                Serial.println("cuadrante " + String(cuadrante));
                new_g1 = bi_interpolation(g1, g2, 0, 0, 100, 100, 0, 100, 100, 100);
                new_g2 = bi_interpolation(g1, g2, 0, 0, 100, 100, 0, 100, 0, 30);

            } else if (g1 > 0 && g2 < 0) {
                cuadrante = 4;
                Serial.println("cuadrante " + String(cuadrante));
                new_g1 = bi_interpolation(g1, g2, 0, -100, 100, 0, -70, 0, 0, 100);
                new_g2 = bi_interpolation(g1, g2, 0, -100, 100, 0, -70, 0, -70, 0);
            } else if (g1 < 0 && g2 > 0) {
                cuadrante = 2;
                Serial.println("cuadrante " + String(cuadrante));
                new_g1 = bi_interpolation(g1, g2, -100, 0, 0, 100, 0, 30, 0, 100);
                new_g2 = bi_interpolation(g1, g2, -100, 0, 0, 100, 100, 100, 0, 100);
            } else {
                cuadrante = 3;
                Serial.println("cuadrante " + String(cuadrante));
                new_g1 = bi_interpolation(g1, g2, -100, -100, 0, 0, -70, 0, -70, 0);
                new_g2 = bi_interpolation(g1, g2, -100, -100, 0, 0, 0, 100, -70, 0);
            }

            g2 = map(new_g1, -100, 100, 0, 1023);
            g1 = map(new_g2, -100, 100, 0, 1023);
            Serial.println("Valor G1: " + String(new_g1) + "Valor G2: " + String(new_g2));
            return;
        }

        if (data1.indexOf("zero") != -1) {
            // Serial.println("zero");
            g1 = 512;
            g2 = 512;
            Xdeviation = 0;

            if (!Autocruise) {
                mandoTX.MotorR = 0;
                mandoTX.MotorL = 0;
            }
        }
        if (data1.indexOf("TR") != -1) {
            // Serial.println("tr");
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.tolba2 = (v == 1) ? 0 : 90;
        }

        if (data1.indexOf("TL") != -1) {
            //  Serial.println("iz");
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.tolba1 = (v == 1) ? 180 : 90;
        }
        if (data1.indexOf("Tolb") != -1) {
            // Serial.println("tt");
            int v = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.tolba2 = (v == 1) ? 0 : 90;
            mandoTX.tolba1 = (v == 1) ? 180 : 90;
        }

        if (data1.indexOf("Gyro") != -1) {
            mandoTX.tolba = Power360;

            timerTolba = 200;
        }

        if (data1.indexOf("Eco") != -1) {
            int v = data1.substring(data1.indexOf(":") + 1).toInt();

            SaveEnergy = (v == 1) ? true : false;
        }

        if (data1.indexOf("gohome:true") != -1) {  // TURN ON AUTOHOME
            mandoTX.Comando = AUTO_HOME;
            mandoTX.commandValue = true;
            autoHome = true;
            return;
        }

        if (data1.indexOf("gohome:false") != -1) {  // TURN OFF AUTOHOME
            mandoTX.Comando = AUTO_HOME;
            mandoTX.commandValue = false;
            autoHome = false;
            return;
        }

        if (data1.indexOf("home") != -1) {
            float lat = data1.substring(data1.indexOf(":") + 1).toFloat();
            float lon = data1.substring(data1.indexOf(",") + 1).toFloat();
            mandoTX.lat = lat;
            mandoTX.lon = lon;
            mandoTX.Comando = SAVE_HOME;
            Home.lat = lat;
            Home.lon = lon;
            dataSaved.lat = lat;
            dataSaved.lon = lon;
            EEPROM.put(0, dataSaved);
            return;
        }

        if (data1.indexOf("go") != -1) {
            float lat = data1.substring(data1.indexOf(":") + 1).toFloat();
            float lon = data1.substring(data1.indexOf(",") + 1).toFloat();
            mandoTX.lat = lat;
            mandoTX.lon = lon;
            mandoTX.Comando = AUTOPILOT;
            Destino.lat = lat;
            Destino.lon = lon;
            autoPilot = true;
            Autocruise = false;
            return;
        }

        if (data1.indexOf("STOP") != -1) {  // STOP Autopilot
            mandoTX.Comando = STOP;
            autoPilot = false;
            autoHome = false;
        }

        if (data1.indexOf("calib:5") != -1) {  // calibracion de brujula manual
            mandoTX.Comando = COMPASS_CAL;
        }

        if (data1.indexOf("calib:1") != -1) {  // calibracion de accelerometro
            mandoTX.Comando = ACCEL_CAL;
        }

        if (data1.indexOf("calib:3") != -1) {  // calibracion de brujula automatica
            mandoTX.Comando = COMPASS_CAL_AUTO;
        }

        if (data1.indexOf("Repeater:true") != -1) {  // turn on repeater
            mandoTX.Comando = WIFI_REPEATER;
            mandoTX.commandValue = true;
        }

        if (data1.indexOf("Repeater:false") != -1) {  // turn off repeater
            mandoTX.Comando = WIFI_REPEATER;
            mandoTX.commandValue = false;
        }

        if (data1.indexOf("OK") != -1) {
            OkTime = millis();
            BtHasConnected = true;
        }

        if (data1.indexOf("invml:true") != -1) {
            mandoTX.Comando = MOTOR_L_INVERSION;
            mandoTX.commandValue = true;
        }

        if (data1.indexOf("invml:false") != -1) {
            mandoTX.Comando = MOTOR_L_INVERSION;
            mandoTX.commandValue = false;
        }

        if (data1.indexOf("invmr:true") != -1) {
            mandoTX.Comando = MOTOR_R_INVERSION;
            mandoTX.commandValue = true;
        }

        if (data1.indexOf("invmr:false") != -1) {
            mandoTX.Comando = MOTOR_R_INVERSION;
            mandoTX.commandValue = false;
        }

        if (data1.indexOf("anglecor") != -1) {
            int angleCor = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Comando = ANGLE_CORRECTION;
            mandoTX.commandValue = angleCor;
        }

        if (data1.indexOf("tempcor") != -1) {
            int tempCor = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Comando = TEMP_CORRECTION;
            mandoTX.commandValue = tempCor;
        }

        if (data1.indexOf("presscor") != -1) {
            int pressCor = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Comando = PRESS_CORRECTION;
            mandoTX.commandValue = pressCor;
        }

        if (data1.indexOf("tolvicor") != -1) {
            int tolvICor = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Comando = TOLVA_L_CORRECTION;
            mandoTX.commandValue = tolvICor;
        }

        if (data1.indexOf("tolvdcor") != -1) {
            int tolvRCor = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Comando = TOLVA_R_CORRECTION;
            mandoTX.commandValue = tolvRCor;
        }

        if (data1.indexOf("initangle") != -1) {
            int initAngleCorrection = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Comando = INIT_ANGLE_CORRECTION;
            mandoTX.commandValue = initAngleCorrection;
        }

        if (data1.indexOf("autofactor") != -1) {
            int autopilotCorrectionFactor = data1.substring(data1.indexOf(":") + 1).toInt();
            mandoTX.Comando = AUTOPILOT_COMPENSATION;
            mandoTX.commandValue = autopilotCorrectionFactor;
        }

        ////SerialBT.flush();
    }
// -------------------------------------------------------------------------------
}

int bi_interpolation(float x, float y, float x1, float y1, float x2, float y2, float q11, float q12, float q21, float q22) {
    float R1 = ((x2 - x) / (x2 - x1)) * q11 + ((x - x1) / (x2 - x1)) * q21;
    float R2 = ((x2 - x) / (x2 - x1)) * q12 + ((x - x1) / (x2 - x1)) * q22;
    float P = ((y2 - y) / (y2 - y1)) * R1 + ((y - y1) / (y2 - y1)) * R2;
    return (int)P;
}



void bleSendTask(void *pvParameters) {
    RadioRXPacket* receivedData = ((RadioRXPacket *)pvParameters);
    int calibCount = 0;
    bool buttonAppState = false;
    uint32_t prevTimeState;
    for (;;)  // A Task shall never return or exit.
    {
        uint8_t apstate = receivedData->apState;

        if ((apstate & 0b00001100) == 0b00000100) {  // if autoHome idle
            apstate = buttonAppState ? (apstate | 0b00001100) & 0b11110111 : apstate & 0b11110011;
        }

        if ((apstate & 0b00000011) == 0b00000001) {  // if autoTolva idle
            apstate = buttonAppState ? (apstate | 0b00000011) & 0b11111101 : apstate & 0b11111100;
        }

        if (xTaskGetTickCount() - prevTimeState > pdMS_TO_TICKS(500)) {
            buttonAppState = !buttonAppState;
            prevTimeState = xTaskGetTickCount();
        }

        ble.write("{\"lat\":");
        ble.write(String(receivedData->lat,6).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->lon, 6);
        ble.write("{\"lon\":");
        ble.write(String(receivedData->lon,6).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->batt1);
        ble.write("{\"b\":");
        ble.write(String(receivedData->batt1).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->batt2);
        ble.write("{\"b1\":");
        ble.write(String(receivedData->batt2).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->satelites);
        ble.write("{\"sat\":");
        ble.write(String(receivedData->satelites).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->gyros);
        ble.write("{\"gyro\":");
        ble.write(String(receivedData->gyros).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->speed);
        ble.write("{\"speed\":");
        ble.write(String((float)receivedData->speed).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->c1);
        ble.write("{\"c1\":");
        ble.write(String((float)receivedData->c1 / 1000.0).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));


        //Serial.println(receivedData->c2);
        ble.write("{\"c2\":");
        ble.write(String((float)receivedData->c2 / 1000.0).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->motorL);
        ble.write("{\"motorl\":");
        ble.write(String(receivedData->motorL).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->motorR);
        ble.write("{\"motorr\":");
        ble.write(String(receivedData->motorR).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->temperatura);
        ble.write("{\"t\":");
        ble.write(String((float)receivedData->temperatura / 100.0).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->presion);
        ble.write("{\"p\":");
        ble.write(String((float)receivedData->presion).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->presion);
        ble.write("{\"calib\":");
        ble.write(String(calibStatus).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        //Serial.println(receivedData->presion);
        ble.write("{\"apState\":");
        ble.write(String(apstate).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));
        
        ble.write("{\"signal\":");
        ble.write(String(dataRate).c_str());
        ble.write("}");
        vTaskDelay(pdMS_TO_TICKS(30));

        if (autoCorr) {     
            ble.write("{\"autoCorr\":");
            ble.write("\"_\"}");
            vTaskDelay(pdMS_TO_TICKS(30));
            autoCorr = false;
        }

        if (boatInit){     
            ble.write("{\"boatInit\":");
            ble.write("\"_\"}");
            vTaskDelay(pdMS_TO_TICKS(30));
            boatInit = false;
        }



        if (calibStatus != 0) {
            calibCount++;
            if (calibCount > 3) {
                calibStatus = 0;
                calibCount = 0;
            }
        }
    }
}

void bleReceiveTask(void *pvParameters) {
    char reply[100];
    char* btData = (char*)pvParameters;

    /*
      Blink
      Turns on an LED on for one second, then off for one second, repeatedly.

      If you want to know what pin the on-board LED is connected to on your ESP32 model, check
      the Technical Specs of your board.
    */

    for (;;)  // A Task shall never return or exit.
    {
        int i = 0;
        while (ble.available()) {
            reply[i] = ble.read();
            i += 1;
        }
        // end the string
        reply[i] = '\0';
        if (strlen(reply) > 0) {
            Serial.println(reply);
            strcpy(btData,reply);
            newData = true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void espNowSendTask(void *pvParameters) {
    RadioTXPacket* currentData = (RadioTXPacket*)pvParameters;

    /*
      Blink
      Turns on an LED on for one second, then off for one second, repeatedly.

      If you want to know what pin the on-board LED is connected to on your ESP32 model, check
      the Technical Specs of your board.
    */

    for (;;)  // A Task shall never return or exit.
    {
        esp_now_send(broadcastAddress, (uint8_t *)currentData, sizeof(mandoTX));
        if (updateSent) {
            strcpy(ssid, wifiData.substring(0, wifiData.indexOf(',')).c_str());
            strcpy(psw, wifiData.substring(wifiData.indexOf(',') + 1).c_str());
            checkUpdate();
            updateSent = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void updateDataRate(int d_value) {
    static uint8_t dataRateArray[CIRCULAR_BUFFER_SIZE];
    static uint8_t avgDataRate = 0;
    static uint8_t dataRateSum = 0;
    static uint8_t circularPointer = 0;

    dataRateSum = dataRateSum - dataRateArray[circularPointer] + d_value;
    dataRateArray[circularPointer] = d_value;
    circularPointer = circularPointer + 1;
    if (circularPointer == CIRCULAR_BUFFER_SIZE) {
        circularPointer = 0;
    }
    avgDataRate = ((float)dataRateSum / (float)CIRCULAR_BUFFER_SIZE) * 100;
    Serial.println(avgDataRate);
    dataRate = avgDataRate;

    if(avgDataRate <= 25){
        digitalWrite(32, HIGH);
        digitalWrite(33, LOW);
        digitalWrite(25, LOW);
        digitalWrite(26, LOW);
    } else if (avgDataRate > 25 && avgDataRate <= 50) {
        digitalWrite(32, HIGH);
        digitalWrite(33, HIGH);
        digitalWrite(25, LOW);
        digitalWrite(26, LOW);
    } else if (avgDataRate > 50 && avgDataRate <= 75) {
        digitalWrite(32, HIGH);
        digitalWrite(33, HIGH);
        digitalWrite(25, HIGH);
        digitalWrite(26, LOW);
    } else if (avgDataRate > 75) {
        digitalWrite(32, HIGH);
        digitalWrite(33, HIGH);
        digitalWrite(25, HIGH);
        digitalWrite(26, HIGH);
    }
}