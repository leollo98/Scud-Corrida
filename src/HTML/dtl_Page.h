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
}

.botao:hover{
    background:#2E7D32;
}

.footer{
    margin-top:15px;
    color:#666;
    font-size:0.9em;
}

</style>

</head>

<body>

<h1>DataLogger ESP32</h1>

<div class="info">
)rawliteral";


const char HTML_FOOTER[] PROGMEM = R"rawliteral(

</table>

<div class="footer">
<a class='botao' href='/reboot'>Reiniciar</a>"
Página gerada pelo ESP32

</div>

</body>

</html>

)rawliteral";