#include "dtl.h"
#include "../espnow/espnow_fragmentado.h"

Adafruit_MPU6050 mpu;
SdFat sd;
SdFile dataFile;
TaskHandle_t Task1;
Preferences prefs;

// file Server
DNSServer dnsServer;
AsyncWebServer server(80);

// mutex
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// SD chip select pin.
const uint8_t SD_CS_PIN = 5;

volatile bool reiniciar = false;
std::string csv = "";
std::string dadosParaGravar = "";
uint32_t linha = 0;

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
  if (!mpu.begin(0x69)) {
#ifdef dev
    Serial.println("Falha ao inicializar o MPU6050.");
    esp_restart();
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

std::string getNextCsvFilename(int8_t offset = 0,
                               const std::string &prefix = "datalogger-") {
  SdFile dir;
  dir.open("/", O_READ);
  SdFile entry;
  int maxNum = 0;
  char name[64];
  prefs.begin("config", true);
  std::string ID = prefs.getString("ID","0").c_str();
  prefs.end();

  while (entry.openNext(&dir, O_READ)) {
    if (entry.isDir()) {
      entry.close();
      continue;
    }

    entry.getName(name, sizeof(name));
    std::string filename(name);

    bool valid = true;

    if (filename.size() <= prefix.size() + 4)
      valid = false;

    std::string ext = filename.substr(filename.size() - 4);
    if (ext != ".csv" && ext != ".CSV")
      valid = false;

    if (filename.rfind(prefix, 0) != 0)
      valid = false;

    if (valid) {
      size_t start = prefix.size();
      size_t end = filename.find_last_of('.');
      std::string numPart = filename.substr(start, end - start);

      int num = atoi(numPart.c_str());
      if (num > maxNum)
        maxNum = num;
    }

    entry.close();
  }

  dir.close();
  return prefix + ID + "_" + std::to_string(maxNum + 1 + offset) + ".csv";
}

void abreArquivo() {
  if (dataFile.isOpen()) {
    dataFile.close();
  }
  if (dataFile.open(getNextCsvFilename(0).c_str(),
                    O_RDWR | O_CREAT | O_AT_END)) {
#ifdef dev
    Serial.println("   -   Arquivo aberto. Salvando dados...");
#endif
  } else if (dataFile.open(getNextCsvFilename(1).c_str(),
                           O_RDWR | O_CREAT | O_AT_END)) {
    /* code */
  } else {
    Serial.begin(115200);
    for (uint16_t i = 0; i < 500; i++) {
      Serial.print("Erro ao abrir o arquivo: ");
      Serial.print(getNextCsvFilename(0).c_str());
      Serial.print(" e ");
      Serial.print(getNextCsvFilename(1).c_str());
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
  portENTER_CRITICAL(&mux);
  csv += std::to_string(millis());
  csv += ",";
  csv += std::to_string(dataFrame.accel[0]);
  csv += ",";
  csv += std::to_string(dataFrame.accel[1]);
  csv += ",";
  csv += std::to_string(dataFrame.accel[2]);
  csv += ",";
  csv += std::to_string(dataFrame.gyro[0]);
  csv += ",";
  csv += std::to_string(dataFrame.gyro[1]);
  csv += ",";
  csv += std::to_string(dataFrame.gyro[2]);
  csv += ",";
  csv += std::to_string(dataFrame.tempC);
  csv += ",";
  csv += std::to_string(dataFrame.presFreio);
  csv += ",";
  csv += std::to_string(dataFrame.pulsosRodaDireita);
  csv += ",";
  csv += std::to_string(dataFrame.pulsosRodaEsquerda);
  csv += "\n";
  portEXIT_CRITICAL(&mux);
}

std::string getLineN(const std::string &dadosParaEnviar, int n) {
  const char *ptr = dadosParaEnviar.c_str();
  const char *start = ptr;
  int linha = 0;

  while (*ptr) {
    if (*ptr == '\n') {
      if (linha == n) {
        // Copia só o trecho da linha (sem '\n')
        return std::string(start, ptr - start);
      }
      linha++;
      start = ptr + 1;
    }
    ptr++;
  }

  // Se chegou aqui, o arquivo acabou antes da linha n
  return "";
}

void core2(void *parameter) {
  unsigned long ultimaTransmissao = 0;
  const unsigned long intervaloEnvio = 100; // 100 ms entre envios

  while (true) {
    if (csv.size() >= pageSize * numberOfPages) {
      portENTER_CRITICAL(&mux);
      digitalWrite(02, 1);
      dadosParaGravar = csv;
      csv = "";
      portEXIT_CRITICAL(&mux);

      dataFile.print(dadosParaGravar.c_str());
      dataFile.flush();
      digitalWrite(02, 0);
      linha = 0;
      Serial.println("gravado");
    } else {
      unsigned long agora = millis();
      if (agora - ultimaTransmissao >= intervaloEnvio) {
        digitalWrite(15, 1);
        ultimaTransmissao = agora;

        std::string envio = getLineN(dadosParaGravar, linha);
        if (!envio.empty()) {
          linha += 25;
          enviarDadosFragmentado(envio);
          // enviarDadosTruncado(envio);
        }
        digitalWrite(15, 0);
      }
    }

    if (dataFile.fileSize() >= 2E9) {
      abreArquivo();
    }
  }
}

void setupPulseCounters() {
  pcnt_config_t pcnt_config;
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DIS;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = 32767;
  pcnt_config.counter_l_lim = 0;

  pcnt_config_t pcnt_config_dir = pcnt_config;
  pcnt_config_dir.pulse_gpio_num = 35;
  pcnt_config_dir.unit = PCNT_UNIT_DIREITA;
  Serial.println(pcnt_unit_config(&pcnt_config_dir));

  pcnt_config_t pcnt_config_esq = pcnt_config;
  pcnt_config_esq.pulse_gpio_num = 32;
  pcnt_config_esq.unit = PCNT_UNIT_ESQUERDA;
  pcnt_unit_config(&pcnt_config_esq);

  // Habilita contadores
  pcnt_counter_pause(PCNT_UNIT_DIREITA);
  pcnt_counter_clear(PCNT_UNIT_DIREITA);
  pcnt_counter_resume(PCNT_UNIT_DIREITA);

  pcnt_counter_pause(PCNT_UNIT_ESQUERDA);
  pcnt_counter_clear(PCNT_UNIT_ESQUERDA);
  pcnt_counter_resume(PCNT_UNIT_ESQUERDA);
}

void pinDef() {
  setupPulseCounters();
  pinMode(4, INPUT_PULLUP);
#ifdef pin
  pinMode(15, OUTPUT);
  pinMode(02, OUTPUT);
#endif
}

void serverSetup() {
  WiFi.mode(WIFI_AP);
  uint8_t baseMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  prefs.begin("config", true);
  String ID = prefs.getString("ID",String(baseMac[5]));
  prefs.end();
  String APname = "DataLogger - " + ID;
  Serial.println(APname);
  const char *AP_NAME = APname.c_str();

  WiFi.softAP(AP_NAME, NULL, 6);

  dnsServer.start(53, "*", WiFi.softAPIP());

  setupWebServer(server);

  server.begin();
}

void ServerLoop() { dnsServer.processNextRequest(); }

void digital() {
  int16_t countDir = 0;
  int16_t countEsq = 0;

  pcnt_get_counter_value(PCNT_UNIT_DIREITA, &countDir);
  pcnt_counter_clear(PCNT_UNIT_DIREITA);

  pcnt_get_counter_value(PCNT_UNIT_ESQUERDA, &countEsq);
  pcnt_counter_clear(PCNT_UNIT_ESQUERDA);

  dataFrame.pulsosRodaDireita = (uint8_t)countDir;
  dataFrame.pulsosRodaEsquerda = (uint8_t)countEsq;
}

void IRAM_ATTR fileServer() { reiniciar = true; }

void readData() {
  prefs.begin("boot", false);
  if (!digitalRead(4) or prefs.getBool("read_data", false)) {
    Serial.println("Servidor de arquivos aberto");
    setupComandos();
    enviarComando("CMD:READ_DATA");
    prefs.putBool("read_data", false);
    prefs.end();
    serverSetup();
    while (true) {
      ServerLoop();
      if (!digitalRead(4)) {
        enviarComando("CMD:READ_DATA");
        delay(300);
      }
      delay(10);
    }
  }
  prefs.end();
}

void Wifi_reset(){
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(100);
}

void setupDTL() {
  Serial.begin(115200);
  pinDef();
  readData();
  Wifi_reset();
  init_componentes();
  configMPU();
  inicializaArquivo();
  setupEspNowSend();
  xTaskCreatePinnedToCore(core2, "core2", 10000, NULL, 0, &Task1, 0);
  attachInterrupt(4, fileServer, FALLING);
}

void loopDTL() {
  MPU();
  analog();
  digital();
  microSD();
  if (reiniciar) {
    reiniciar = false;
    ESP.restart();
  }
}
