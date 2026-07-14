#pragma once
#include <Arduino.h>

const char HTML_HEADER[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">

<head>

<meta charset="utf-8">

<meta name="viewport" content="width=device-width, initial-scale=1">

<title>DataLogger ESP32</title>

<style>

*{
    box-sizing:border-box;
}

body{
    font-family:Arial,Helvetica,sans-serif;
    background:#f2f2f2;
    margin:20px;
    color:#333;
}

h1{
    margin-bottom:15px;
}

.info{
    background:#ffffff;
    padding:15px;
    border-radius:8px;
    margin-bottom:20px;
    box-shadow:0 1px 3px rgba(0,0,0,0.15);
}

table{
    width:100%;
    border-collapse:collapse;
    table-layout:auto;
    background:#ffffff;
    box-shadow:0 1px 3px rgba(0,0,0,0.15);
}

th{
    background:#1976D2;
    color:white;
    padding:10px;
}

td{
    padding:10px;
    border-bottom:1px solid #dddddd;
    white-space:nowrap;
}

tr:nth-child(even){
    background:#f7f7f7;
}

th:nth-child(1),
td:nth-child(1){
    width:70px;
    text-align:center;
}

th:nth-child(3),
td:nth-child(3){
    width:120px;
    text-align:right;
}

th:nth-child(4),
td:nth-child(4){
    width:120px;
    text-align:center;
}

.botao{
    display:inline-block;
    padding:8px 14px;
    background:#4CAF50;
    color:white;
    text-decoration:none;
    border-radius:4px;
    margin-right:10px;
}

.botao:hover{
    background:#2E7D32;
}

.reboot{
    background:#d32f2f;
}

.reboot:hover{
    background:#b71c1c;
}

.footer{
    margin-top:15px;
    color:#666;
    font-size:0.9em;
}

.menu{
    margin-bottom:15px;
    display:flex;
    gap:10px;
    flex-wrap:wrap;
}

</style>

</head>

<body>

<h1>DataLogger ESP32</h1>

<div class="menu">
    <a class="botao" href="/">Arquivos</a>
    <a class="botao" href="/config">Configurações</a>
    <a class="botao" href="/update">Atualização OTA</a>
    <a class="botao reboot" href="/reboot">Reiniciar</a>
</div>

<div class="info">

<div class="info">
)rawliteral";

const char HTML_FOOTER[] PROGMEM = R"rawliteral(

</table>

<div class="footer">
Página gerada pelo ESP32
</div>

</body>

</html>

)rawliteral";

const char HTML_CONFIG[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Configuração</title>

<style>
body{
    font-family: Arial, sans-serif;
    background:#f0f0f0;
    display:flex;
    justify-content:center;
    align-items:center;
    height:100vh;
    margin:0;
}

.container{
    background:white;
    padding:25px;
    border-radius:10px;
    box-shadow:0 0 10px rgba(0,0,0,.2);
    width:320px;
}

h2{
    text-align:center;
}

label{
    display:block;
    margin-top:15px;
    font-weight:bold;
}

input[type=text]{
    width:100%;
    padding:10px;
    margin-top:5px;
    box-sizing:border-box;
}

button{
    width:100%;
    margin-top:20px;
    padding:10px;
    background:#2196F3;
    color:white;
    border:none;
    border-radius:5px;
    cursor:pointer;
}

button:hover{
    background:#1976D2;
}
</style>

</head>

<body>

<div class="container">
    <h2>Configuração</h2>

    <form method="POST" action="/config">
        <label for="ID">Local</label>
        <input
            type="text"
            id="ID"
            name="ID"
            placeholder="Digite o local"
            required>

        <button type="submit">Salvar</button>
    </form>
</div>

</body>
</html>
)rawliteral";

const char HTML_UPDATE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">

<head>

<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">

<title>Atualização OTA</title>

<style>

*{
    box-sizing:border-box;
}

body{
    font-family:Arial,Helvetica,sans-serif;
    background:#f2f2f2;
    margin:30px;
    color:#333;
}

.card{
    max-width:600px;
    margin:auto;
    background:#fff;
    padding:25px;
    border-radius:8px;
    box-shadow:0 2px 8px rgba(0,0,0,.15);
}

h2{
    margin-top:0;
}

input[type=file]{
    width:100%;
    padding:8px;
    margin:15px 0;
}

button{
    background:#1976D2;
    color:#fff;
    border:none;
    padding:10px 18px;
    border-radius:4px;
    cursor:pointer;
    font-size:15px;
}

button:hover{
    background:#1565C0;
}

button:disabled{
    background:#999;
    cursor:not-allowed;
}

.bar{
    width:100%;
    height:24px;
    background:#ddd;
    border-radius:12px;
    overflow:hidden;
    margin-top:20px;
}

#fill{
    width:0%;
    height:100%;
    background:#4CAF50;
    transition:width .2s;
}

.info{
    margin-top:15px;
    line-height:1.7em;
}

.alerta{
    margin-top:20px;
    color:#d32f2f;
    font-weight:bold;
}

.ok{
    color:#2E7D32;
}

</style>

</head>

<body>

<div class="card">

<h2>Atualização OTA</h2>

<input
    type="file"
    id="firmware"
    accept=".bin">

<button id="upload">
Atualizar Firmware
</button>

<div class="bar">
    <div id="fill"></div>
</div>

<div class="info">

<div>
<b>Progresso:</b>
<span id="percent">0%</span>
</div>

<div>
<b>Enviado:</b>
<span id="size">0 / 0 kB</span>
</div>

<div>
<b>Velocidade:</b>
<span id="speed">0 kB/s</span>
</div>

<div id="status">
Aguardando seleção do firmware...
</div>

</div>

<div class="alerta">
⚠ Não desligue o ESP32 durante a atualização.
</div>

</div>

<script>

const uploadBtn = document.getElementById("upload");
const firmware = document.getElementById("firmware");

const fill = document.getElementById("fill");
const percent = document.getElementById("percent");
const size = document.getElementById("size");
const speed = document.getElementById("speed");
const status = document.getElementById("status");

uploadBtn.onclick = function(){

    if(firmware.files.length==0){
        alert("Selecione um firmware (.bin).");
        return;
    }
    window.onbeforeunload = function () {
        return "Atualização em andamento.";
    };

    uploadBtn.disabled = true;

    let file = firmware.files[0];

    let form = new FormData();
    form.append("update", file);

    let xhr = new XMLHttpRequest();

    let start = Date.now();

    xhr.open("POST","/update",true);

    xhr.upload.onprogress = function(e){

        if(!e.lengthComputable)
            return;

        let p = Math.round(e.loaded*100/e.total);

        fill.style.width = p + "%";

        percent.innerHTML = p + "%";

        size.innerHTML =
            (e.loaded/1024).toFixed(1) +
            " / " +
            (e.total/1024).toFixed(1) +
            " kB";

        let elapsed = (Date.now()-start)/1000;

        let kbps = (e.loaded/1024)/elapsed;

        speed.innerHTML =
            kbps.toFixed(1) +
            " kB/s";

        status.innerHTML = "Enviando firmware...";
        
    };

    xhr.onload = function(){
        window.onbeforeunload = null;
        if(xhr.status >= 200 && xhr.status < 300){

            fill.style.background="#2E7D32";

            status.innerHTML =
                "<span class='ok'><b>✔ Firmware atualizado com sucesso.</b><br>Reiniciando o ESP32...</span>";

        }else{

            fill.style.background="#d32f2f";

            status.innerHTML =
                "<span style='color:#d32f2f'><b>Erro durante a atualização.</b></span>";

            uploadBtn.disabled = false;

        }

    };

    xhr.onerror = function(){
        window.onbeforeunload = null;
        fill.style.background="#d32f2f";

        status.innerHTML =
            "<span style='color:#d32f2f'><b>Falha de comunicação.</b></span>";

        uploadBtn.disabled = false;

    };

    xhr.send(form);

};

</script>

</body>

</html>
)rawliteral";