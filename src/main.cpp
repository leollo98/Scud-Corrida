#include <Arduino.h>
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Scuderia UFABC //
// Universidade Federal do ABC //
// datalogger_rm03_v1 //
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware: Esp32-WROOM-DA Module;
// Módulo MicroSD
// MPU6050
// DS3231
// 5 x Potenciômetros lineares de 10k ohms
// 2 x Potenciômetros lineares de 01k ohms
// Software: Arduino IDE 2.2.2
// Library: Adafruit BusIO 1.14.5
// Adafruit MPU6050 2.2.6
// Adafruit Unified Sensor 1.1.14
// RTClib 2.1.3
// Última atualização: 07 de janeiro de 2024 por Carlos Saravia
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#define chipSelectPin 5 // chip select (CS) do módulo do cartão SD
#define presFreDiaPin 36
#define presFreTraPin 39
#define interval 5
// #define dev
// #define time

Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;
File dataFile;
DateTime now;
TaskHandle_t Task1;

struct dados {
  /// Define variáveis globais para armazenar os dados do MPU6050
  float accel[3];
  float gyro[3];
  float tempC;
  /// Define variáveis globais para armazenar os dados da Susp
  uint32_t presFreDia = 0;
  uint32_t presFreTra = 0;
  boolean novoDado = false;
};
dados dataFrame;
dados dataFrame2Write;

uint32_t previousMillis = 0;

void init_componentes() {
  if (!rtc.begin()) {
#ifdef dev
    Serial.println("Falha ao inicializar o RTC");
    // esp_restart();
#endif
  }
  if (!mpu.begin(
          0x69)) { // Endereço definido apenas com VCC no pino AD0 do MPU6050
#ifdef dev
    Serial.println("Falha ao inicializar o MPU6050.");
    // esp_restart();
#endif
  }
  if (!SD.begin(chipSelectPin)) {
    Serial.begin(115200);
    for (uint16_t i = 0; i < 500; i++) {
      Serial.println("Falha ao inicializar o cartão SD.");
    }
    esp_restart();
  }
}

void configMPU() {
  mpu.setAccelerometerRange(
      MPU6050_RANGE_4_G); // Define a faixa de medição do acelerômetro
  mpu.setGyroRange(
      MPU6050_RANGE_250_DEG); // Define a faixa de medição do giroscópio
  mpu.setFilterBandwidth(
      MPU6050_BAND_94_HZ); // Define a largura de banda do filtro do sensor
}

String nomeArquivo() {
  // Criar uma string para armazenar o nome do arquivo
  String fileName = "/datalogger_";
  // Obter a data e hora atual do RTC
  DateTime now = rtc.now();
  // Adicionar ano, mês, dia, hora, minuto e segundo ao nome do arquivo
  fileName += String(now.year());
  fileName += "_";
  fileName += String(now.month());
  fileName += "_";
  fileName += String(now.day());
  fileName += "_";
  fileName += String(now.hour());
  fileName += "_";
  fileName += String(now.minute());
  fileName += "_";
  fileName += String(now.second());
  fileName += ".csv";
  return fileName;
}

void inicializaArquivo() {
  dataFile = SD.open(nomeArquivo(), FILE_WRITE);
  if (dataFile) {
#ifdef dev
    Serial.println("Arquivo aberto. Salvando dados...");
#endif
  } else {
    Serial.begin(115200);
    for (uint16_t i = 0; i < 500; i++) {
      Serial.println("Erro ao abrir o arquivo.");
    }
    esp_restart();
  }
  dataFile.print("tempo de execucao,aceleracao x,aceleracao y,aceleracao "
                 "z,giroscopio x,giroscopio y,giroscopio z,temperatura "
                 "mpu,pressao freio dianteiro,pressao freio traseiro");
  dataFile.println();
  dataFile.flush();
}

void verificaRTC() {
  if (rtc.lostPower()) {
#ifdef dev
    Serial.println("RTC perdeu energia. Defina a hora novamente.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
#endif
  }
  // Ajuste manual do horário do RTC DS3231
  // Para adiantar 1 minuto:
  // DateTime now = rtc.now();
  // rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(),
  // now.minute() + 1, now.second())); Ou para atrasar 1 minuto: DateTime now =
  // rtc.now(); rtc.adjust(DateTime(now.year(), now.month(), now.day(),
  // now.hour(), now.minute() - 1, now.second()));
}

void RTC() {
  // Atualiza a variável global 'now' com a data e hora atuais
  now = rtc.now();
  DateTime now = rtc.now();
#ifdef dev
  Serial.print("RTC:");
  Serial.print(now.year());
  Serial.print(" ");
  Serial.print(now.month());
  Serial.print(" ");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(" ");
  Serial.print(now.minute());
  Serial.print(" ");
  Serial.print(now.second());
  Serial.print(" ");
#endif
}

void MPU() {
  sensors_event_t accelerometer, gyroscope, temperature;
  mpu.getEvent(&accelerometer, &gyroscope, &temperature);
  // Armazenar os valores lidos nas variáveis globais
  dataFrame.accel[0] = accelerometer.acceleration.x;
  dataFrame.accel[1] = accelerometer.acceleration.y;
  dataFrame.accel[2] = accelerometer.acceleration.z;
  dataFrame.gyro[0] = gyroscope.gyro.x;
  dataFrame.gyro[1] = gyroscope.gyro.y;
  dataFrame.gyro[2] = gyroscope.gyro.z;
  dataFrame.tempC = temperature.temperature;
#ifdef dev
  // Mostra valores de aceleração em m/s²
  Serial.print("acl:");
  Serial.print(accel[0]);
  Serial.print(" ");
  Serial.print(accel[1]);
  Serial.print(" ");
  Serial.print(accel[2]);
  Serial.print(" ");
  // Mostra valores do giroscópio em rad/s
  Serial.print("g:");
  Serial.print(gyro[0]);
  Serial.print(" ");
  Serial.print(gyro[1]);
  Serial.print(" ");
  Serial.print(gyro[2]);
  Serial.print(" ");
  // Mostra valor da temperatura da placa em degC
  Serial.print("T:");
  Serial.print(tempC);
  Serial.print(" ");
#endif
}

void Freios() {
  dataFrame.presFreDia =
      analogRead(presFreDiaPin); // Lê o valor do potenciômetro da pressão de
                                 // fluido de freio dianteiro
  dataFrame.presFreTra =
      analogRead(presFreTraPin); // Lê o valor do potenciômetro da pressão de
                                 // fluido de freio traseiro
#ifdef dev
  Serial.print("F:");
  Serial.print(presFreDia);
  Serial.print(" ");
  Serial.println(presFreTra);
#endif
}

void microSD() {
#ifdef time
  Serial.print("   start: ");
  Serial.print(micros());
#endif
  dataFile.print(millis());
  dataFile.print(',');
#ifdef time
  Serial.print("   2m: ");
  Serial.print(micros());
#endif
  for (uint8_t i = 0; i < 3; i++) {
    dataFile.print(dataFrame.accel[i]);
    dataFile.print(',');
  }
  for (uint8_t i = 0; i < 3; i++) {
    dataFile.print(dataFrame.gyro[i]);
    dataFile.print(',');
  }
#ifdef time
  Serial.print("   for: ");
  Serial.print(micros());
#endif
  dataFile.print(dataFrame.tempC);
  dataFile.print(',');
  dataFile.print(dataFrame.presFreDia);
  dataFile.print(',');
  dataFile.print(dataFrame.presFreTra);
  dataFile.print(',');
  dataFile.println();
#ifdef time
  Serial.print("   resto: ");
  Serial.print(micros());
#endif
  dataFile.flush();
#ifdef time
  Serial.print("   flush: ");
  Serial.println(micros());
#endif
}

void core2(void *parameter) {
  while (true) {
    while (dataFrame2Write.novoDado == false) {
      NOP();
    }
    microSD();
    dataFrame2Write.novoDado = false;
  }
}

void setup() {
#if defined(dev) || defined(time)
  Serial.begin(115200);
#endif
  init_componentes();
  configMPU();
  verificaRTC();
  inicializaArquivo();
  xTaskCreatePinnedToCore(core2, "core2", 10000, NULL, 0, &Task1, 0);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    MPU();
    Freios();
    while (dataFrame2Write.novoDado == true) {
      NOP();
    }
    dataFrame2Write = dataFrame;
    dataFrame2Write.novoDado = true;
  }
}
