/* 
  FSWebServer - Example WebServer with SPIFFS backend for esp8266
  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the ESP8266WebServer library for Arduino environment.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  upload the contents of the data folder with MkSPIFFS Tool ("ESP8266 Sketch Data Upload" in Tools menu in Arduino IDE)
  or you can upload the contents of a folder if you CD in that folder and run the following command:
  for file in `ls -A1`; do curl -F "file=@$PWD/$file" esp8266fs.local/edit; done
  
  access the sample web page at http://esp8266fs.local
  edit the page by going to http://esp8266fs.local/edit
*/
/*
 * This file is part of the esp8266 web interface
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <Ticker.h>

#include <SD_MMC.h>
#include "RTClib.h"
#include <ESP32Time.h>
#include <time.h>
#include "driver/uart.h"

#define DBG_OUTPUT_PORT Serial
#define INVERTER_PORT UART_NUM_2
#define INVERTER_RX 16
#define INVERTER_TX 17
#define UART_TIMEOUT (100 / portTICK_PERIOD_MS)
#define UART_MESSBUF_SIZE 100
#define LED_BUILTIN 13 //clashes with SDIO, need to change to suit hardware and uncomment lines

#define RESERVED_SD_SPACE 31000000000 //2000000000
#define SDIO_BUFFER_SIZE 16384
#define FLUSH_WRITES 60 //flush file every 60 blocks

//HardwareSerial Inverter(INVERTER_PORT);

const char* host = "inverter";
bool fastUart = false;
bool fastUartAvailable = true;
char uartMessBuff[UART_MESSBUF_SIZE];

WebServer server(80);
HTTPUpdateServer updater;
//holds the current upload
File fsUploadFile;
Ticker sta_tick;

//SWD over ESP8266
/*
  https://github.com/scanlime/esp8266-arm-swd
*/
#include "src/arm_debug.h"
#include <StreamString.h>
uint32_t addr = 0x08000000;
uint32_t addrEnd = 0x0801ffff;
const uint8_t swd_clock_pin = 4; //GPIO4 (D2)
const uint8_t swd_data_pin = 5; //GPIO5 (D1)
ARMDebug swd(swd_clock_pin, swd_data_pin, ARMDebug::LOG_NONE);

RTC_PCF8523 ext_rtc;
ESP32Time int_rtc;
uint32_t nextFileIndex = 0;
bool haveRTC = false;
bool haveSDCard = false;
bool fastLoggingEnabled = true;
bool fastLoggingActive = false;
uint8_t SDIObuffer[SDIO_BUFFER_SIZE];
uint16_t indexSDIObuffer = 0;
uint16_t blockCountSD = 0;
File dataFile;
int startLogAttempt = 0;

bool createNextSDFile()
{
  char filename[50];
  if(haveRTC)
    nextFileIndex = 0; //have a date so restart index from 0 (still needed in case serial stream fails to start)

  do
  {
    if(haveRTC)
      snprintf(filename, 50, "/%d-%02d-%02d-%02d-%02d-%02d_%d.bin", int_rtc.getYear(), int_rtc.getMonth(), int_rtc.getDay(), int_rtc.getHour(), int_rtc.getMinute(), int_rtc.getSecond(), nextFileIndex++);
    else
      snprintf(filename, 50, "/%010d.bin", nextFileIndex++);
  }
  while(SD_MMC.exists(filename));
      
  dataFile = SD_MMC.open(filename, FILE_WRITE);
  if (dataFile) 
  {
    dataFile.flush(); //make sure FAT updated for debugging purposes
    DBG_OUTPUT_PORT.println("Created file: " + String(filename)); 
    return true;
  }
  else
    return false;
}

uint32_t deleteOldest(uint64_t spaceRequired)
{
  time_t oldestTime = 0;
  File root, file;
  String oldestFileName;
  uint64_t spaceRem;
  time_t t;
  uint32_t nextIndex = 0;
  
  spaceRem = SD_MMC.totalBytes() - SD_MMC.usedBytes();

  DBG_OUTPUT_PORT.println("Space Required = " + formatBytes(spaceRequired));
  DBG_OUTPUT_PORT.println("Space Remaining = " + formatBytes(spaceRem));
  
  while(spaceRem < spaceRequired)
  {
    root = SD_MMC.open("/");
    
    oldestTime = 0;
    while(file = root.openNextFile())
    {
      if(haveRTC)
        t = file.getLastWrite();
      else
      {
        String fname = file.name();
        fname.remove(0,1); //lose starting /
        t = fname.toInt()+1; //make sure 0 special case isnt used
        if(t > nextIndex)
          nextIndex = t;
      }
      if(!file.isDirectory() && ((oldestTime==0) || (t<oldestTime)))
      {
        oldestTime = t;
        oldestFileName = "/";
        oldestFileName += file.name();
      }
      file.close();
    }  
    root.close();

    
    if(oldestFileName.length() > 0)
    {
      
      if(SD_MMC.remove(oldestFileName))
        DBG_OUTPUT_PORT.println("Deleted file: " + oldestFileName);
      else
        DBG_OUTPUT_PORT.println("Couldn't delete: " + oldestFileName);
    }
    else
    {
      DBG_OUTPUT_PORT.println("No files found, can't free space");
      break;//no files so can do no more
    }
  }
  return(nextIndex);
}

//format bytes
String formatBytes(uint64_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
  //DBG_OUTPUT_PORT.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.html";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload(){
  if(server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    //DBG_OUTPUT_PORT.print("handleFileUpload Name: "); DBG_OUTPUT_PORT.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    //DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    //DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
  }
}

void handleFileDelete(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  //DBG_OUTPUT_PORT.println("handleFileDelete: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate(){
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  //DBG_OUTPUT_PORT.println("handleFileCreate: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  String path = "/";
  if(server.hasArg("dir")) 
    String path = server.arg("dir");
  //DBG_OUTPUT_PORT.println("handleFileList: " + path);
  File root = SPIFFS.open(path);
  String output = "[";

  if(!root){
    //DBG_OUTPUT_PORT.print("- failed to open directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += file.isDirectory()?"dir":"file";
    output += "\",\"name\":\"";
    output += String(file.name()).substring(1);
    output += "\"}";
    file = root.openNextFile();
  }
  
  output += "]";
  server.send(200, "text/json", output);
}

// static void sendCommand(String cmd)
// {
//   DBG_OUTPUT_PORT.println("Sending '" + cmd + "' to inverter");
//   Inverter.print("\n");
//   delay(1);
//   while(Inverter.available())
//     Inverter.read(); //flush all previous output
//   Inverter.print(cmd);
//   Inverter.print("\n");
//   Inverter.readStringUntil('\n'); //consume echo  
// }

void uart_readUntill(char val)
{
  int retVal;
  do
  {
    retVal = uart_read_bytes(UART_NUM_2, uartMessBuff, 1, UART_TIMEOUT);
  }
  while((retVal>0) && (uartMessBuff[0] != val));
}

bool uart_readStartsWith(const char *val)
{
  bool retVal = false;
  int rxBytes = uart_read_bytes(UART_NUM_2, uartMessBuff, strnlen(val,UART_MESSBUF_SIZE), UART_TIMEOUT);
  if(rxBytes >= strnlen(val,UART_MESSBUF_SIZE))
  {
    if(strncmp(val, uartMessBuff, strnlen(val,UART_MESSBUF_SIZE))==0)
      retVal = true;
    uartMessBuff[rxBytes] = 0;
    DBG_OUTPUT_PORT.println(uartMessBuff);
  }
  return retVal;
}



static void sendCommand(String cmd)
{
  DBG_OUTPUT_PORT.println("Sending '" + cmd + "' to inverter");
  //Inverter.print("\n");
  uart_write_bytes(INVERTER_PORT, "\n", 1);
  delay(1);
  //while(Inverter.available())
  //  Inverter.read(); //flush all previous output
  uart_flush(INVERTER_PORT);
  //Inverter.print(cmd);
  uart_write_bytes(INVERTER_PORT, cmd.c_str(), cmd.length());
  //Inverter.print("\n");
  uart_write_bytes(INVERTER_PORT, "\n", 1);
  //Inverter.readStringUntil('\n'); //consume echo  
  uart_readUntill('\n');
}

static void handleCommand() {
  const int cmdBufSize = 128;
  if(!server.hasArg("cmd")) {server.send(500, "text/plain", "BAD ARGS"); return;}

  String cmd = server.arg("cmd").substring(0, cmdBufSize);
  int repeat = 0;
  char buffer[255];
  size_t len = 0;
  String output;

  if (server.hasArg("repeat"))
    repeat = server.arg("repeat").toInt();

  if (!fastUart && fastUartAvailable)
  {
    sendCommand("fastuart");
    if (uart_readStartsWith("OK"))
    {
      //Inverter.begin(921600, SERIAL_8N1, INVERTER_RX, INVERTER_TX);
      //Inverter.updateBaudRate(921600);
      uart_set_baudrate(INVERTER_PORT, 921600);
      fastUart = true;
    }
    else
    {
      fastUartAvailable = false;
    }
  }

  sendCommand(cmd);
  do {
    memset(buffer,0,sizeof(buffer));
    //len = Inverter.readBytes(buffer, sizeof(buffer) - 1);
    len = uart_read_bytes(UART_NUM_2, buffer, sizeof(buffer), UART_TIMEOUT);
    if(len > 0) output.concat(buffer, len);// += buffer;

    if (repeat)
    {
      repeat--;
      //Inverter.print("!");
      uart_write_bytes(INVERTER_PORT, "!", 1);
      //Inverter.readBytes(buffer, 1); //consume "!"
      uart_read_bytes(UART_NUM_2, buffer, 1, UART_TIMEOUT);
    }
  } while (len > 0);
  DBG_OUTPUT_PORT.println(output);
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.send(200, "text/json", output);
}

static uint32_t crc32_word(uint32_t Crc, uint32_t Data)
{
  int i;

  Crc = Crc ^ Data;

  for(i=0; i<32; i++)
    if (Crc & 0x80000000)
      Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
    else
      Crc = (Crc << 1);

  return(Crc);
}

static uint32_t crc32(uint32_t* data, uint32_t len, uint32_t crc)
{
   for (uint32_t i = 0; i < len; i++)
      crc = crc32_word(crc, data[i]);
   return crc;
}


static void handleUpdate()
{
  if(!server.hasArg("step") || !server.hasArg("file")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  size_t PAGE_SIZE_BYTES = 1024;
  int step = server.arg("step").toInt();
  File file = SPIFFS.open(server.arg("file"), "r");
  int pages = (file.size() + PAGE_SIZE_BYTES - 1) / PAGE_SIZE_BYTES;
  String message;

  if (server.hasArg("pagesize"))
  {
    PAGE_SIZE_BYTES = server.arg("pagesize").toInt();
  }

  if (step == -1)
  {
    //int c;
    char c;
    sendCommand("reset");

    if (fastUart)
    {
      //Inverter.begin(115200, SERIAL_8N1, INVERTER_RX, INVERTER_TX);
      //Inverter.updateBaudRate(115200);
      uart_set_baudrate(INVERTER_PORT, 115200);
      fastUart = false;
      fastUartAvailable = true; //retry after reboot
    }
    do {
      //c = Inverter.read();
      uart_read_bytes(INVERTER_PORT, &c, 1, UART_TIMEOUT);
    } while (c != 'S' && c != '2');

    if (c == '2') //version 2 bootloader
    {
      //Inverter.write(0xAA); //Send magic
      c = 0xAA;
      uart_write_bytes(INVERTER_PORT, &c, 1);
      //while (Inverter.read() != 'P');
      do {
        uart_read_bytes(INVERTER_PORT, &c, 1, UART_TIMEOUT);
      } while (c != 'S');
    }
    
    //Inverter.write(pages);
    snprintf(uartMessBuff, UART_MESSBUF_SIZE, "%d", pages);
    uart_write_bytes(INVERTER_PORT, uartMessBuff, strnlen(uartMessBuff, UART_MESSBUF_SIZE));
    //while (Inverter.read() != 'P');
    do {
      uart_read_bytes(INVERTER_PORT, &c, 1, UART_TIMEOUT);
    } while (c != 'P');
    message = "reset";
  }
  else
  {
    bool repeat = true;
    file.seek(step * PAGE_SIZE_BYTES);
    char buffer[PAGE_SIZE_BYTES];
    size_t bytesRead = file.readBytes(buffer, sizeof(buffer));

    while (bytesRead < PAGE_SIZE_BYTES)
      buffer[bytesRead++] = 0xff;
    
    uint32_t crc = crc32((uint32_t*)buffer, PAGE_SIZE_BYTES / 4, 0xffffffff);

    while (repeat)
    {
      //Inverter.write(buffer, sizeof(buffer));
      uart_write_bytes(INVERTER_PORT, buffer, sizeof(buffer));
      //while (!Inverter.available());
      char res;// = Inverter.read();
      while(uart_read_bytes(INVERTER_PORT, &res, 1, UART_TIMEOUT)<=0);

      if ('C' == res) {
        //Inverter.write((char*)&crc, sizeof(uint32_t));
        uart_write_bytes(INVERTER_PORT, (char*)&crc, sizeof(uint32_t));
        //while (!Inverter.available());
        //res = Inverter.read();
        while(uart_read_bytes(INVERTER_PORT, &res, 1, UART_TIMEOUT)<=0);
      }

      switch (res) {
        case 'D':
          message = "Update Done";
          repeat = false;
          fastUartAvailable = true;
          break;
        case 'E':
          //while (Inverter.read() != 'T');
          do {
            uart_read_bytes(INVERTER_PORT, uartMessBuff, 1, UART_TIMEOUT);
          } while (uartMessBuff[0] != 'T');
          break;
        case 'P':
          message = "Page write success";
          repeat = false;
          break;
        default:
        case 'T':
          break;
      }
    }
  }
  server.send(200, "text/json", "{ \"message\": \"" + message + "\", \"pages\": " + pages + " }");
  file.close();
}

static void handleWifi()
{
  bool updated = true;
  if(server.hasArg("apSSID") && server.hasArg("apPW")) 
  {
    WiFi.softAP(server.arg("apSSID").c_str(), server.arg("apPW").c_str());
  }
  else if(server.hasArg("staSSID") && server.hasArg("staPW")) 
  {
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(server.arg("staSSID").c_str(), server.arg("staPW").c_str());
  }
  else
  {
    File file = SPIFFS.open("/wifi.html", "r");
    String html = file.readString();
    file.close();
    html.replace("%staSSID%", WiFi.SSID());
    html.replace("%apSSID%", WiFi.softAPSSID());
    html.replace("%staIP%", WiFi.localIP().toString());
    server.send(200, "text/html", html);
    updated = false;
  }

  if (updated)
  {
    File file = SPIFFS.open("/wifi-updated.html", "r");
    size_t sent = server.streamFile(file, getContentType("wifi-updated.html"));
    file.close();    
  }
}

static void handleBaud()
{
  if (fastUart)
    server.send(200, "text/html", "fastUart on");
  else
    server.send(200, "text/html", "fastUart off");
}

void staCheck(){
  sta_tick.detach();
  if(!(uint32_t)WiFi.localIP()){
    WiFi.mode(WIFI_AP); //disable station mode
  }
}

void setup(void){
  DBG_OUTPUT_PORT.begin(115200);
  //Inverter.setRxBufferSize(50000);
  //Inverter.begin(115200, SERIAL_8N1, INVERTER_RX, INVERTER_TX);
  //Need to use low level Espressif IDF API instead of Serial to get high enough data rates
  uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

  uart_param_config(INVERTER_PORT, &uart_config);
  uart_set_pin(INVERTER_PORT, INVERTER_TX, INVERTER_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(INVERTER_PORT, SDIO_BUFFER_SIZE * 3, 0, 0, NULL, 0); //x3 allows twice card write size to buffer while writes
  delay(100);        

  //check for external RTC and if present use to initialise on-chip RTC
  if (ext_rtc.begin())
  {
    haveRTC = true;
    DBG_OUTPUT_PORT.println("External RTC found");  
    if (! ext_rtc.initialized() || ext_rtc.lostPower()) 
    {
      DBG_OUTPUT_PORT.println("RTC is NOT initialized, setting to build time");
      ext_rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    ext_rtc.start();
    DateTime now = ext_rtc.now();
    int_rtc.setTime(now.unixtime());  
  }
  else
    DBG_OUTPUT_PORT.println("No RTC found, defaulting to sequential file names"); 

  //initialise SD card in SDIO mode
  if (SD_MMC.begin()) 
  {
    DBG_OUTPUT_PORT.println("Started SD_MMC");  
    nextFileIndex = deleteOldest(RESERVED_SD_SPACE);
    haveSDCard = true;    
  }
  else
    DBG_OUTPUT_PORT.println("Couldn't start SD_MMC");  

  //Start SPI Flash file system
  SPIFFS.begin();

  //WIFI INIT
  #ifdef WIFI_IS_OFF_AT_BOOT
    enableWiFiAtBootTime();
  #endif
  WiFi.mode(WIFI_AP_STA);
  //WiFi.setPhyMode(WIFI_PHY_MODE_11B);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);//25); //dbm
  WiFi.begin();
  sta_tick.attach(10, staCheck);
  
  MDNS.begin(host);

  updater.setup(&server);
  
  //SERVER INIT
  ArduinoOTA.setHostname(host);
  ArduinoOTA.begin();
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload);

  server.on("/wifi", handleWifi);
  server.on("/cmd", handleCommand);
  server.on("/fwupdate", handleUpdate);
  server.on("/baud", handleBaud);
  server.on("/version", [](){ server.send(200, "text/plain", "1.1.R"); });
  server.on("/swd/begin", []() {
    // See if we can communicate. If so, return information about the target.
    // This shouldn't reset the target, but it does need to communicate,
    // and the debug port itself will be reset.
    //
    // If all is well, this returns some identifying info about the target.

    uint32_t idcode;

    if (swd.begin() && swd.getIDCODE(idcode)) {

      char output[128];
      snprintf(output, sizeof output, "{\"connected\": true, \"idcode\": \"0x%02x\" }", idcode);
      server.send(200, "application/json", String(output));

    } else {
      server.send(200, "application/json", "{\"connected\": false}");
    }
  });
  server.on("/swd/uid", []() {
    // STM32F103 Reference Manual, Chapter 30.2 Unique device ID register (96 bits)
    // http://www.st.com/st-web-ui/static/active/en/resource/technical/document/reference_manual/CD00171190.pdf

    uint32_t REG_U_ID = 0x1FFFF7E8; //96 bits long, read using 3 read operations

    uint16_t off0;
    uint16_t off2;
    uint32_t off4;
    uint32_t off8;

    swd.memLoadHalf(REG_U_ID + 0x0, off0);
    swd.memLoadHalf(REG_U_ID + 0x2, off2);
    swd.memLoad(REG_U_ID + 0x4, off4);
    swd.memLoad(REG_U_ID + 0x8, off8);

    char output[128];
    snprintf(output, sizeof output, "{\"uid\": \"0x%04x-0x%04x-0x%08x-0x%08x\" }", off0, off2, off4, off8);
    server.send(200, "application/json", String(output));
  });
  server.on("/swd/halt", []() {
    if (swd.begin()) {
      char output[128];
      snprintf(output, sizeof output, "{\"halt\": \"%s\"}", swd.debugHalt() ? "true" : "false");
      server.send(200, "application/json", String(output));
    } else {
      server.send(200, "text/plain", "SWD Error");
    }
  });
  server.on("/swd/run", []() {
    if (swd.begin()) {
      char output[128];
      snprintf(output, sizeof output, "{\"run\": \"%s\"}", swd.debugRun() ? "true" : "false");
      server.send(200, "application/json", String(output));
    } else {
      server.send(200, "text/plain", "SWD Error");
    }
  });
  server.on("/swd/reset", []() {
    if (swd.begin()) {
      bool debugHalt = swd.debugHalt();
      bool debugReset = false;
      if (server.hasArg("hard")) {
        swd.reset();
        debugReset = true;
      } else {
        debugReset = swd.debugReset();
      }
      char output[128];
      snprintf(output, sizeof output, "{\"halt\": \"%s\", \"reset\": \"%s\"}", debugHalt ? "true" : "false", debugReset ? "true" : "false");
      server.send(200, "application/json", String(output));
    } else {
      server.send(200, "text/plain", "SWD Error");
    }
  });
  server.on("/swd/zero", []() {

    char output[128];

    if (swd.begin()) {

      uint32_t addrTotal = addrEnd - addr;
      server.setContentLength(CONTENT_LENGTH_UNKNOWN);
      server.send(200, "text/plain", "");

      swd.debugHalt();
      swd.debugHaltOnReset(1);
      swd.reset();
      swd.unlockFlash();

      //METHOD #1
      swd.flashEraseAll();

      //METHOD #2
      // Before programming internal SRAM, the ARM Cortex-M3 should first be reset and halted.
      /*
        1. Write 0xA05F0003 to DHCSR. This will halt the core.
        2. Write 1 to bit VC_CORERESET in DEMCR. This will enable halt-on-reset
        3. Write 0xFA050004 to AIRCR. This will reset the core.
      */
      //swd.flashloaderSRAM();

      uint32_t addrNext = addr;
      uint32_t addrIndex = 0;
      uint32_t addrBuffer = 0x00000000; //Used by METHOD #2
      do {
        //Serial.printf("------ %08x -> %08x ------\n", addrNext, addrBuffer);

        snprintf(output, sizeof output, "%08x:", addrNext);
        server.sendContent(output);

        uint32_t eraseBuffer[4];
        memset(eraseBuffer, 0xff, sizeof(eraseBuffer));

        for (int i = 0; i < 4; i++)
        {
          //METHOD #2
          //swd.writeBufferSRAM(addrBuffer, eraseBuffer, 1);

          //METHOD #3
          //swd.flashWrite(addrNext, eraseBuffer[i]);

          snprintf(output, sizeof output, " | %02x %02x %02x %02x", (uint8_t)(eraseBuffer[i] >> 0), (uint8_t)(eraseBuffer[i] >> 8), (uint8_t)(eraseBuffer[i] >> 16), (uint8_t)(eraseBuffer[i] >> 24));
          server.sendContent(output);

          addrNext += 4;
          addrBuffer += 4;
        }

        server.sendContent("\n");

        addrIndex++;
      } while (addrNext <= addrEnd);

      //METHOD #2
      //swd.flashloaderRUN(addr, addrBuffer);

      swd.debugHaltOnReset(0);
      swd.debugReset();

      server.sendContent(""); //end stream

    } else {
      server.send(200, "text/plain", "SWD Error");
    }
  });
  server.on("/swd/hex", []() {

    if (swd.begin()) {

      if (server.hasArg("bootloader")) {
        addr = 0x08000000;
        addrEnd = 0x08000fff;
      } else if (server.hasArg("flash")) {
        addr = 0x08001000;
        addrEnd = 0x0801ffff;
        //addrEnd = 0x080011ff; //Quick Debug
      } else if (server.hasArg("ram")) {
        addr = 0x20000000;
        addrEnd = 0x200003ff; //Note: Read is limited to 0x200003ff but you can write to higher portion of RAM
      }
      server.setContentLength(CONTENT_LENGTH_UNKNOWN);
      server.send(200, "text/plain", "");

      uint32_t addrCount = 256;
      uint32_t addrNext = addr;
      do {

        //Serial.printf("------ %08x ------\n", addrNext);

        StreamString data;
        swd.hexDump(addrNext, addrCount, data);
        server.sendContent(data.readString());

        addrNext += (addrCount * 4); //step = count * 4 bytes in int32 word
      } while (addrNext <= addrEnd);

      server.sendContent(""); //end stream

    } else {
      server.send(200, "text/plain", "SWD Error");
    }
  });
  server.on("/swd/bin", []() {

    if (swd.begin()) {

      String filename = "flash.bin";

      if (server.hasArg("bootloader")) {
        addr = 0x08000000;
        addrEnd = 0x08000fff;
        filename = "bootloader.bin";
      } else if (server.hasArg("flash")) {
        addr = 0x08001000;
        addrEnd = 0x0801ffff;
      }
      
      server.sendHeader("Content-Disposition", "attachment; filename = \"" + filename + "\"");
      server.setContentLength(addrEnd - addr + 1); //CONTENT_LENGTH_UNKNOWN
      server.send(200, "application/octet-stream", "");

      uint32_t addrNext = addr;
      do {

        //Serial.printf("------ %08x ------\n", addrNext);

        //uint8_t* buff;
        //swd.binDump(addrNext, buff);
        //server.sendContent(String((char *)buff));
        
        uint8_t byte;
        swd.memLoadByte(addrNext, byte);
        server.sendContent(String(byte));

        addrNext++;
      } while (addrNext <= addrEnd);

      server.sendContent(""); //end stream

    } else {
      server.send(200, "text/plain", "SWD Error");
    }
  });
  server.on("/swd/mem/flash", []() {

    char output[128];

    if (swd.begin()) {

      if (server.hasArg("file")) {

        if (server.hasArg("bootloader")) {
          addr = 0x08000000;
          addrEnd = 0x08000fff;
        } else if (server.hasArg("flash")) {
          addr = 0x08001000;
          addrEnd = 0x0801ffff;
        }

        String filename = server.arg("file");
        File fs = SPIFFS.open("/" + filename, "r");
        if (fs)
        {
          server.setContentLength(CONTENT_LENGTH_UNKNOWN);
          server.send(200, "text/plain", "");

          swd.debugHalt();
          swd.debugHaltOnReset(1); //reset lock into halt
          swd.reset();
          swd.unlockFlash();

          //pinMode(LED_BUILTIN, OUTPUT);

          uint32_t addrNext = addr;
          uint32_t addrIndex = addr;
          uint32_t addrBuffer = 0x00000000;

          while (addrNext < addrEnd && fs.available())
          {
            swd.debugHalt();
            if (addrBuffer == 0x00000000)
            {
              swd.flashloaderSRAM(); //load flashloader to SRAM @ 0x20000000
            }

            //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            
            uint8_t PAGE_SIZE = 6; //webserver max chunks
            for (uint8_t p = 0; p < PAGE_SIZE; p++)
            {
              //Serial.printf("------ %08x ------\n", addrIndex);
              if (fs.available() == 0)
                break;

              snprintf(output, sizeof output, "%08x:", addrIndex);
              server.sendContent(output);

              for (int i = 0; i < 4; i++)
              {
                if (fs.available() == 0)
                  break;

                char sramBuffer[4];
                fs.readBytes(sramBuffer, 4);
                swd.writeBufferSRAM(addrBuffer, (uint8_t*)sramBuffer, sizeof(sramBuffer)); //append to SRAM after flashloader

                snprintf(output, sizeof output, " | %02x %02x %02x %02x", sramBuffer[0], sramBuffer[1], sramBuffer[2], sramBuffer[3]);
                server.sendContent(output);

                addrIndex += 4;
                addrBuffer += 4;
              }
              server.sendContent("\n");
            }
            swd.flashloaderRUN(addrNext, addrBuffer);
            delay(400); //Must wait for flashloader to finish
 
            addrBuffer = 0x00000000;
            addrNext = addrIndex;
          }

          swd.debugHaltOnReset(0); //no reset halt lock
          swd.reset(); //hard-reset

          fs.close();
          SPIFFS.remove("/" + filename);

          server.sendContent(""); //end stream
          //digitalWrite(LED_BUILTIN, HIGH); //OFF
        } else {
          server.send(200, "text/plain", "File Error");
        }
      } else {
        server.send(200, "text/plain", ".bin File Required");
      }
    } else {
      server.send(200, "text/plain", "SWD Error");
    }
  });
  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound([](){
    if(!handleFileRead(server.uri()))
    {
      server.sendHeader("Refresh", "6; url=/update");
      server.send(404, "text/plain", "FileNotFound");
    }
  });

  server.begin();
  server.client().setNoDelay(1);

  MDNS.addService("http", "tcp", 80);
}
 
void loop(void){
  // note: ArduinoOTA.handle() calls MDNS.update();
  server.handleClient();
  ArduinoOTA.handle();

  if(haveSDCard && fastLoggingEnabled)  
  {
    if(WiFi.softAPgetStationNum() == 0) //no connected stations so do fast debug
    {
      if(fastLoggingActive) //already active, just carry on writing data
      {
        int spaceAvail = SDIO_BUFFER_SIZE - indexSDIObuffer;
        int bytesRead = uart_read_bytes(INVERTER_PORT, &SDIObuffer[indexSDIObuffer], spaceAvail, UART_TIMEOUT);
        if(bytesRead > 0)
        {
          indexSDIObuffer += bytesRead;
          if(indexSDIObuffer >= SDIO_BUFFER_SIZE)
          {
            dataFile.write(SDIObuffer, SDIO_BUFFER_SIZE);
            indexSDIObuffer = 0;
            blockCountSD++;
            if(blockCountSD >= FLUSH_WRITES)
            {
              blockCountSD = 0;
              dataFile.flush();
            }
          }
        }
      }
      else //not active so start
      {
        if(startLogAttempt < 3)
        {
          startLogAttempt++;
          if(createNextSDFile())
          {
            sendCommand(""); //flush out buffer in case just had power up
            delay(10);
            sendCommand("binarylogging 1"); //send start logging command to inverter
            delayMicroseconds(200);
            if (uart_readStartsWith("OK"))
            {
              uart_set_baudrate(INVERTER_PORT, 2250000);
              fastLoggingActive = true;
              DBG_OUTPUT_PORT.println("Binary logging started");
            }
            else //no response - in case it did actually switch but we missed response send the turn off command
            {
              dataFile.close();
              uart_set_baudrate(INVERTER_PORT, 2250000);
              uart_write_bytes(INVERTER_PORT, "\n", 1);
              delay(1);
              uart_write_bytes(INVERTER_PORT, "binarylogging 0", strnlen("binarylogging 0", UART_MESSBUF_SIZE));
              uart_write_bytes(INVERTER_PORT, "\n", 1);
              uart_wait_tx_done(INVERTER_PORT, UART_TIMEOUT);
              uart_set_baudrate(INVERTER_PORT, 115200);
            }
            delay(10);
            uart_flush(INVERTER_PORT);
          }
        }
      }
    }
    else
    {
      startLogAttempt=0; //restart log attempts when next disconnected
      if(fastLoggingActive) //was it active last pass
      {
        uart_write_bytes(INVERTER_PORT, "\n", 1);
        delay(1);
        uart_write_bytes(INVERTER_PORT, "binarylogging 0", strnlen("binarylogging 0", UART_MESSBUF_SIZE));
        uart_write_bytes(INVERTER_PORT, "\n", 1);
        uart_wait_tx_done(INVERTER_PORT, UART_TIMEOUT);
        uart_set_baudrate(INVERTER_PORT, 115200);
        delay(10);
        uart_flush(INVERTER_PORT);
        //data should now have stopped so send command again and check response
        sendCommand("binarylogging 0");
        if (uart_readStartsWith("OK"))
        {
          uart_set_baudrate(INVERTER_PORT, 115200);
          fastUart = false;
          fastLoggingActive = false;
          dataFile.flush(); //make sure up to date
          dataFile.close();
          DBG_OUTPUT_PORT.println("Binary logging terminated");
        }
        else
        { //assume still logging so try again next time round
          uart_set_baudrate(INVERTER_PORT, 2250000);
        }
        delay(10);
        uart_flush(INVERTER_PORT);
      }
    }
  }    
}
