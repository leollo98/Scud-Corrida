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
#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>

#define chipSelectPin 5 // chip select (CS) do módulo do cartão SD
#define pinPresFreio 34
// #define dev
#define pin
//  1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 1

Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;
SdFat sd;
SdFile dataFile;
DateTime now;
TaskHandle_t Task1;

// SD chip select pin.
const uint8_t SD_CS_PIN = 5;

// Use a large percent of sector size for best performance (512B -> 2K -> 4k).
#define numeroDeLinhasParaGravar 24 // 2048 > dadosCSV * numeroDeLinhasParaGravar < 1024
uint8_t microsdVezesParaEscrita = 0;
std::string csv;

struct dados {
  float accel[3];
  float gyro[3];
  float tempC;
  uint32_t presFreio = 0;
  uint8_t pulsosRodaDireita = 0;
  uint8_t pulsosRodaEsquerda = 0;
};
dados dataFrame;

uint32_t previousMillis = 0;

void init_componentes() {
  if (!rtc.begin()) {
#ifdef dev
    Serial.println("Falha ao inicializar o RTC");
    // esp_restart();
#endif
  }
  if (!mpu.begin(0x69)) {
#ifdef dev
    Serial.println("Falha ao inicializar o MPU6050.");
    // esp_restart();
#endif
  }
  Serial.println("speed:");
  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(20)))) {
    Serial.print("20");
    return;
  }
  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(18)))) {
    Serial.println("18");
    return;
  }
  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16)))) {
    Serial.print("16");
    return;
  }
  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(14)))) {
    Serial.print("14");
    return;
  }
  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(12)))) {
    Serial.print("12");
    return;
  }
  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(10)))) {
    Serial.print("10");
    return;
  }
  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(8)))) {
    Serial.print("8");
    return;
  }
  for (uint16_t i = 0; i < 500; i++) {
    Serial.println("Falha ao inicializar o cartão SD.");
  }
  esp_restart();
}

void configMPU() {
  mpu.setAccelerometerRange(
      MPU6050_RANGE_4_G); // Define a faixa de medição do acelerômetro
  mpu.setGyroRange(
      MPU6050_RANGE_500_DEG); // Define a faixa de medição do giroscópio
  mpu.setFilterBandwidth(
      MPU6050_BAND_260_HZ); // Define a largura de banda do filtro do sensor
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

void abreArquivo() {
  if (dataFile.isOpen()) {
    dataFile.close();
  }
  if (dataFile.open(nomeArquivo().c_str(), O_RDWR | O_CREAT | O_AT_END)) {
#ifdef dev
    Serial.println("   -   Arquivo aberto. Salvando dados...");
#endif
  } else {
    Serial.begin(115200);
    for (uint16_t i = 0; i < 500; i++) {
      Serial.print("Erro ao abrir o arquivo: ");
      Serial.println(nomeArquivo().c_str());
    }
    esp_restart();
  }
}

void inicializaArquivo() {

  abreArquivo();
  dataFile.print("tempo de execucao,aceleracao x,aceleracao y,aceleracao "
                 "z,giroscopio x,giroscopio y,giroscopio z,temperatura "
                 "mpu,pressao freio,pulsos roda direita,pulsos roda esquerda");
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
  Serial.print(dataFrame.accel[0]);
  Serial.print(" ");
  Serial.print(dataFrame.accel[1]);
  Serial.print(" ");
  Serial.print(dataFrame.accel[2]);
  Serial.print(" ");
  // Mostra valores do giroscópio em rad/s
  Serial.print("g:");
  Serial.print(dataFrame.gyro[0]);
  Serial.print(" ");
  Serial.print(dataFrame.gyro[1]);
  Serial.print(" ");
  Serial.print(dataFrame.gyro[2]);
  Serial.print(" ");
  // Mostra valor da temperatura da placa em degC
  Serial.print("T:");
  Serial.print(dataFrame.tempC);
  Serial.print(" ");
#endif
}

void analog() {
  dataFrame.presFreio = analogRead(pinPresFreio);
#ifdef dev
  Serial.print("F:");
  Serial.println(dataFrame.presFreio);
#endif
}

void microSD() {
  csv += std::to_string(millis());            // 10 posições
  csv += ",";                                 // 1 posições
  csv += std::to_string(dataFrame.accel[0]);  // 5 posições
  csv += ",";                                 // 1 posições
  csv += std::to_string(dataFrame.accel[1]);  // 5 posições
  csv += ",";                                 // 1 posições
  csv += std::to_string(dataFrame.accel[2]);  // 5 posições
  csv += ",";                                 // 1 posições
  csv += std::to_string(dataFrame.gyro[0]);   // 5 posições
  csv += ",";                                 // 1 posições
  csv += std::to_string(dataFrame.gyro[1]);   // 5 posições
  csv += ",";                                 // 1 posições
  csv += std::to_string(dataFrame.gyro[2]);   // 5 posições
  csv += ",";                                 // 1 posições
  csv += std::to_string(dataFrame.tempC);     // 5 posições
  csv += ",";                                 // 1 posições
  csv += std::to_string(dataFrame.presFreio); // 5 posições
  csv += "\n";                                // 2 posições
  csv += std::to_string(dataFrame.pulsosRodaDireita); // 2 posições
  csv += "\n";                                // 2 posições
  csv += std::to_string(dataFrame.pulsosRodaDireita); // 2 posições
  csv += "\n";                                // 2 posições
  // total de 64 caracteres
  microsdVezesParaEscrita = microsdVezesParaEscrita + 1;
}


void IRAM_ATTR pulsosRodaDireita()
{
  dataFrame.pulsosRodaDireita = dataFrame.pulsosRodaDireita + 1 ;
}
void IRAM_ATTR pulsosRodaEsquerda()
{
  dataFrame.pulsosRodaEsquerda = dataFrame.pulsosRodaEsquerda + 1 ;
}

void core2(void *parameter) {
  while (true) {
    std::string data;
    if (microsdVezesParaEscrita >= numeroDeLinhasParaGravar) {
	  digitalWrite(15, 1);
      data = csv;
      csv = "";
      microsdVezesParaEscrita = 0;
      dataFile.print(data.c_str());
      dataFile.flush();
	  digitalWrite(15, 0);
    }
	if (dataFile.fileSize()>=2E9)
	{
		abreArquivo();
	}
	
  }
}

void setup() {
  Serial.begin(115200);
  init_componentes();
  configMPU();
  verificaRTC();
  inicializaArquivo();
  xTaskCreatePinnedToCore(core2, "core2", 10000, NULL, 0, &Task1, 0);
  Wire.setClock(1000000);
  pinMode(35,PULLDOWN);
  pinMode(32,PULLDOWN);
  attachInterrupt(35, pulsosRodaDireita, RISING);
  attachInterrupt(32, pulsosRodaEsquerda, RISING);
#ifdef pin
  pinMode(15, OUTPUT);
  pinMode(25, OUTPUT);
#endif
}

void loop() {
  
  digitalWrite(25, 1);
  MPU();
  analog();
  digitalWrite(25, 0);
  while (microsdVezesParaEscrita >= numeroDeLinhasParaGravar) {
    NOP();
  }
  microSD();
  
}
