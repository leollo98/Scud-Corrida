#include "dtl_server.h"

std::vector<LogFile> files;
Preferences prefs_server;

String formatBytes(uint64_t bytes) {
  if (bytes < 1024)
    return String(bytes) + " B";

  if (bytes < 1024 * 1024)
    return String(bytes / 1024.0, 1) + " KB";

  if (bytes < 1024ULL * 1024ULL * 1024ULL)
    return String(bytes / 1024.0 / 1024.0, 2) + " MB";

  return String(bytes / 1024.0 / 1024.0 / 1024.0, 2) + " GB";
}

void updateFileList() {

  files.clear();
  files.reserve(50);

  File root = SD.open("/");

  if (!root) {
    Serial.println("Erro abrindo raiz SD");
    return;
  }

  uint32_t timeout = millis();

  File file = root.openNextFile();

  while (file) {

    if (millis() - timeout > 2000) {
      Serial.println("Timeout lendo SD");
      break;
    }

    String name = file.name();

    if (name.startsWith("/"))
      name.remove(0, 1);

    if (name.startsWith("datalogger") && name.endsWith(".csv")) {

      LogFile log;

      strncpy(log.filename, name.c_str(), sizeof(log.filename));
      log.filename[sizeof(log.filename) - 1] = '\0';
      log.size = file.size();

      files.push_back(log);
    }

    file.close();

    delay(1);

    file = root.openNextFile();
  }

  root.close();
}

void writeHeader(AsyncResponseStream *response) {

  response->print(HTML_HEADER);

  response->printf("<b>Total de arquivos:</b> %u<br>"
                   "<b>Espaço utilizado:</b> %s<br>"
                   "<b>Espaço livre:</b> %s<br>",
                   files.size(), formatBytes(SD.usedBytes()).c_str(),
                   formatBytes(SD.totalBytes() - SD.usedBytes()).c_str());

  response->print("<br>");

  response->print("<table>"
                  "<tr>"
                  "<th>#</th>"
                  "<th>Nome</th>"
                  "<th>Tamanho</th>"
                  "<th>Download</th>"
                  "</tr>");
}

void writeFooter(AsyncResponseStream *response) {
  response->print(HTML_FOOTER);
}

void writeTable(AsyncResponseStream *response){
  response->print(HTML_TABLE);
}

void writeHTML(AsyncResponseStream *response, String html) {
  response->print(html);
}

void handleFiles(AsyncWebServerRequest *request) {
  updateFileList();
  AsyncResponseStream *response =
      request->beginResponseStream("application/json");

  response->print("[");

  bool first = true;

  for (const auto &file : files) {

    if (!first)
      response->print(",");

    first = false;

    response->printf("{\"name\":\"%s\",\"size\":%lu}", file.filename,
                     file.size);
  }

  response->print("]");

  request->send(response);
}

void handleRoot(AsyncWebServerRequest *request) {
  updateFileList();

  AsyncResponseStream *response = request->beginResponseStream("text/html");

  writeHeader(response);
  writeTable(response);
  writeFooter(response);

  request->send(response);
}

void handleDownload(AsyncWebServerRequest *request) {
  if (!request->hasParam("file")) {
    request->send(400, "text/plain", "Arquivo não informado");
    return;
  }

  String filename = request->getParam("file")->value();

  if (!SD.exists("/" + filename)) {
    request->send(404, "text/plain", "Arquivo não encontrado");
    return;
  }

  AsyncWebServerResponse *response =
      request->beginResponse(SD, "/" + filename, "text/csv");

  response->addHeader("Content-Disposition",
                      "attachment; filename=\"" + filename + "\"");

  request->send(response);
}

void handleReboot(AsyncWebServerRequest *request) { esp_restart(); }

void handleConfig(AsyncWebServerRequest *request) {

  prefs_server.begin("config", true);
  String ID = prefs_server.getString("ID", "");
  prefs_server.end();

  String html = HTML_CONFIG;
  html.replace("%ID%", ID);

  AsyncResponseStream *response = request->beginResponseStream("text/html");

  writeHTML(response, html);

  request->send(response);
}

void handleConfigSave(AsyncWebServerRequest *request) {

  if (!request->hasParam("ID", true)) {
    request->send(400, "text/plain", "Campo Local não informado");
    return;
  }

  String ID = request->getParam("ID", true)->value();

  prefs_server.begin("config", false);
  prefs_server.putString("ID", ID);
  prefs_server.end();

  request->redirect("/");
}

void handleNotFound(AsyncWebServerRequest *request) { request->redirect("/"); }

void handleUploadRequest(AsyncWebServerRequest *request) {
  bool ok = !Update.hasError();

  if (ok) {
    request->send(204); // No Content

    Serial.println("OTA concluído. Reiniciando...");
    prefs_server.begin("boot", false);
    prefs_server.putBool("read_data", true);

    delay(500);
    ESP.restart();
  } else {
    request->send(500, "text/plain", "Falha na atualização.");
  }
}

void handleUpdate(AsyncWebServerRequest *request) {
  request->send(200, "text/html", HTML_UPDATE);
}

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index,
                  uint8_t *data, size_t len, bool final) {

  if (!index) {
    Serial.printf("OTA: %s\n", filename.c_str());

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  }

  if (len) {
    if (Update.write(data, len) != len) {
      Update.printError(Serial);
    }
  }

  if (final) {
    if (Update.end(true)) {
      Serial.println("OTA concluído.");
    } else {
      Update.printError(Serial);
    }
  }
}

void setupWebServer(AsyncWebServer &server) {

  if (!SD.begin(5)) {
    Serial.println("Falha ao montar SD");
    esp_restart();
  }
  Serial.println("SD Montado");

  server.on("/download", HTTP_GET, handleDownload);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/config", HTTP_POST, handleConfigSave);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/generate_204", HTTP_GET, handleRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRoot);
  server.on("/fwlink", HTTP_GET, handleRoot);
  server.on("/connecttest.txt", HTTP_GET, handleRoot);
  server.on("/ncsi.txt", HTTP_GET, handleRoot);

  server.onNotFound(handleNotFound);
  server.on("/reboot", HTTP_GET, handleReboot);
  server.on("/update", HTTP_GET, handleUpdate);
  server.on("/update", HTTP_POST, handleUploadRequest, handleUpload);
  server.on("/files", HTTP_GET, handleFiles);
}