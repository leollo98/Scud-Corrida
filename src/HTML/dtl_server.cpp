#include "dtl_server.h"

std::vector<LogFile> files;

String formatBytes(uint64_t bytes) {
  if (bytes < 1024)
    return String(bytes) + " B";

  if (bytes < 1024 * 1024)
    return String(bytes / 1024.0, 1) + " KB";

  if (bytes < 1024ULL * 1024ULL * 1024ULL)
    return String(bytes / 1024.0 / 1024.0, 2) + " MB";

  return String(bytes / 1024.0 / 1024.0 / 1024.0, 2) + " GB";
}

uint32_t getFileNumber(const String &name) {

  int pos = name.indexOf('_');

  if (pos < 0)
    return 0;

  String num = name.substring(pos + 1);

  int dot = num.indexOf('.');

  if (dot >= 0)
    num = num.substring(0, dot);

  return num.toInt();
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

    if (name.startsWith("datalogger_") && name.endsWith(".csv")) {

      LogFile log;

      strncpy(log.filename, name.c_str(), sizeof(log.filename));
      log.filename[sizeof(log.filename) - 1] = '\0';

      log.number = getFileNumber(name);
      log.size = file.size();

      files.push_back(log);
    }

    file.close();

    delay(1);

    file = root.openNextFile();
  }

  root.close();

  std::sort(files.begin(), files.end(),
    [](const LogFile &a, const LogFile &b) {
      return a.number > b.number;
    });
}

void writeTable(AsyncResponseStream *response) {

  for (const auto &file : files) {
    String size = formatBytes(file.size);
    response->printf(
        "<tr>"
        "<td>%lu</td>"
        "<td>%s</td>"
        "<td>%s</td>"
        "<td><a class='botao' href='/download?file=%s'>Baixar</a></td>"
        "</tr>",
        file.number, file.filename, size.c_str(), file.filename);
  }
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

void handleReboot(AsyncWebServerRequest *request){
    esp_restart();
}

void setupWebServer(AsyncWebServer &server) {

  if (!SD.begin(5)) {
    Serial.println("Falha ao montar SD");
    esp_restart();
  }
  Serial.println("SD Montado");

  server.on("/download", HTTP_GET, handleDownload);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/generate_204", HTTP_GET, handleRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRoot);
  server.on("/fwlink", HTTP_GET, handleRoot);
  server.on("/connecttest.txt", HTTP_GET, handleRoot);
  server.on("/ncsi.txt", HTTP_GET, handleRoot);

  server.onNotFound(
      [](AsyncWebServerRequest *request) { request->redirect("/"); });
  server.on("/reboot", HTTP_GET, handleReboot);
}