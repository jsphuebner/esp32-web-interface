/*
 * This file is part of the esp32 web interface
 *
 * Copyright (C) 2023 Johannes Huebner <dev@johanneshuebner.com>
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
#include "driver/gpio.h"
#include "driver/twai.h"
#include <FS.h>
#include <SPIFFS.h> 
#include <StreamUtils.h>
#include <ArduinoJson.h>
#include "oi_can.h"

#define DBG_OUTPUT_PORT Serial
#define SDO_REQUEST_DOWNLOAD  (1 << 5)
#define SDO_REQUEST_UPLOAD    (2 << 5)
#define SDO_REQUEST_SEGMENT   (3 << 5)
#define SDO_TOGGLE_BIT        (1 << 4)
#define SDO_RESPONSE_UPLOAD   (2 << 5)
#define SDO_RESPONSE_DOWNLOAD (3 << 5)
#define SDO_EXPEDITED         (1 << 1)
#define SDO_SIZE_SPECIFIED    (1)
#define SDO_WRITE             (SDO_REQUEST_DOWNLOAD | SDO_EXPEDITED | SDO_SIZE_SPECIFIED)
#define SDO_READ              SDO_REQUEST_UPLOAD
#define SDO_ABORT             0x80
#define SDO_WRITE_REPLY       SDO_RESPONSE_DOWNLOAD
#define SDO_READ_REPLY        (SDO_RESPONSE_UPLOAD | SDO_EXPEDITED | SDO_SIZE_SPECIFIED)
#define SDO_ERR_INVIDX        0x06020000
#define SDO_ERR_RANGE         0x06090030
#define SDO_ERR_GENERAL       0x08000000

#define SDO_INDEX_PARAMS      0x2000
#define SDO_INDEX_PARAM_UID   0x2100
#define SDO_INDEX_MAP_START   0x3000
#define SDO_INDEX_MAP_END     0x4800
#define SDO_INDEX_MAP_RX      0x4000
#define SDO_INDEX_SERIAL      0x5000
#define SDO_INDEX_STRINGS     0x5001
#define SDO_INDEX_COMMANDS    0x5002
#define SDO_CMD_SAVE          0
#define SDO_CMD_LOAD          1
#define SDO_CMD_RESET         2


namespace OICan {

enum state { IDLE, ERROR, OBTAINSERIAL, OBTAIN_JSON };
enum updstate { UPD_IDLE, SEND_MAGIC, SEND_SIZE, SEND_PAGE, CHECK_CRC, REQUEST_JSON };

static uint8_t _nodeId;
static state state;
static updstate updstate;
static uint32_t serial[4]; //contains id sum as well
static char jsonFileName[20];
static twai_message_t tx_frame;
static File updateFile;
static int currentPage = 0;
static const size_t PAGE_SIZE_BYTES = 1024;
static int retries = 0;

static void requestSdoElement(uint16_t index, uint8_t subIndex) {
  tx_frame.extd = false;
  tx_frame.identifier = 0x600 | _nodeId;
  tx_frame.data_length_code = 8;
  tx_frame.data[0] = SDO_READ;
  tx_frame.data[1] = index & 0xFF;
  tx_frame.data[2] = index >> 8;
  tx_frame.data[3] = subIndex;
  tx_frame.data[4] = 0;
  tx_frame.data[5] = 0;
  tx_frame.data[6] = 0;
  tx_frame.data[7] = 0;

  twai_transmit(&tx_frame, pdMS_TO_TICKS(10));
}

static void setValueSdo(uint16_t index, uint8_t subIndex, double value) {
  tx_frame.extd = false;
  tx_frame.identifier = 0x600 | _nodeId;
  tx_frame.data_length_code = 8;
  tx_frame.data[0] = SDO_WRITE;
  tx_frame.data[1] = index & 0xFF;
  tx_frame.data[2] = index >> 8;
  tx_frame.data[3] = subIndex;
  *(int32_t*)&tx_frame.data[4] = value * 32;

  twai_transmit(&tx_frame, pdMS_TO_TICKS(10));
}

static int getId(String name) {
  DynamicJsonDocument doc(300);
  StaticJsonDocument<200> filter;
  
  File file = SPIFFS.open(jsonFileName, "r");
  filter[name]["id"] = true;
  deserializeJson(doc, file, DeserializationOption::Filter(filter));
  file.close();

  return doc[name]["id"].as<int>();
}

static void requestNextSegment(bool toggleBit) {
  tx_frame.extd = false;
  tx_frame.identifier = 0x600 | _nodeId;
  tx_frame.data_length_code = 8;
  tx_frame.data[0] = SDO_REQUEST_SEGMENT | toggleBit << 4;
  tx_frame.data[1] = 0;
  tx_frame.data[2] = 0;
  tx_frame.data[3] = 0;
  tx_frame.data[4] = 0;
  tx_frame.data[5] = 0;
  tx_frame.data[6] = 0;
  tx_frame.data[7] = 0;
  
  twai_transmit(&tx_frame, pdMS_TO_TICKS(10));
}

static void handleSdoResponse(twai_message_t *rxframe) {
  static bool toggleBit = false;
  static File file;
  
  if (rxframe->data[0] == SDO_ABORT) { //SDO abort
    state = ERROR;
    DBG_OUTPUT_PORT.println("Error obtaining serial number, try restarting");
    return;
  }
  
  switch (state) {
    case OBTAINSERIAL:
      if ((rxframe->data[1] | rxframe->data[2] << 8) == SDO_INDEX_SERIAL && rxframe->data[3] < 4) {
        serial[rxframe->data[3]] = *(uint32_t*)&rxframe->data[4];
        
        if (rxframe->data[3] < 3) {
          requestSdoElement(SDO_INDEX_SERIAL, rxframe->data[3] + 1);
        }
        else {
          sprintf(jsonFileName, "/%x.json", serial[3]);
          DBG_OUTPUT_PORT.printf("Got Serial Number %X:%X:%X:%X\r\n", serial[0], serial[1], serial[2], serial[3]);
          
          if (SPIFFS.exists(jsonFileName)) {
            state = IDLE;
            DBG_OUTPUT_PORT.println("json file already downloaded");
          }
          else {
            state = OBTAIN_JSON;
            DBG_OUTPUT_PORT.printf("Downloading json to %s\r\n", jsonFileName);
            file = SPIFFS.open(jsonFileName, "w+");
            requestSdoElement(SDO_INDEX_STRINGS, 0); //Initiates JSON upload
          }
        }
      }
      break;
    case OBTAIN_JSON:
      //Receiving last segment
      if ((rxframe->data[0] & 1) && (rxframe->data[0] & SDO_READ) == 0) {
        int size = 7 - ((rxframe->data[0] >> 1) & 0x7);
        file.write(&rxframe->data[1], size);
        file.close();
        DBG_OUTPUT_PORT.println("Download complete");
        state = IDLE;
      }
      //Receiving a segment
      else if (rxframe->data[0] == (toggleBit << 4) && (rxframe->data[0] & SDO_READ) == 0) {
        file.write(&rxframe->data[1], 7);
        toggleBit = !toggleBit;
        requestNextSegment(toggleBit);
      }
      //Request first segment
      else if ((rxframe->data[0] & SDO_READ) == SDO_READ) {
        requestNextSegment(toggleBit);
      }

      break;
  }
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

static void handleUpdate(twai_message_t *rxframe) {
  static int currentByte = 0;
  static uint32_t crc;
        
  switch (updstate) {
    case SEND_MAGIC:
      if (rxframe->data[0] == 0x33) {
        tx_frame.identifier = 0x7dd;
        tx_frame.data_length_code = 4;

        //For now just reflect ID
        tx_frame.data[0] = rxframe->data[4];
        tx_frame.data[1] = rxframe->data[5];
        tx_frame.data[2] = rxframe->data[6];
        tx_frame.data[3] = rxframe->data[7];
        updstate = SEND_SIZE;
        DBG_OUTPUT_PORT.printf("Sending ID %u\r\n", *(uint32_t*)tx_frame.data);
        twai_transmit(&tx_frame, pdMS_TO_TICKS(10));
      }
      break;
    case SEND_SIZE:
      if (rxframe->data[0] == 'S') {
        tx_frame.identifier = 0x7dd;
        tx_frame.data_length_code = 1;

        tx_frame.data[0] = (updateFile.size() + PAGE_SIZE_BYTES - 1) / PAGE_SIZE_BYTES;
        updstate = SEND_PAGE;
        crc = 0xFFFFFFFF;
        DBG_OUTPUT_PORT.printf("Sending size %u\r\n", tx_frame.data[0]);
        twai_transmit(&tx_frame, pdMS_TO_TICKS(10));
      }
      break;
    case SEND_PAGE:
      if (rxframe->data[0] == 'P') {
        char buffer[8];
        size_t bytesRead = 0;
        
        if (currentByte < updateFile.size()) {
          updateFile.seek(currentByte);
          bytesRead = updateFile.readBytes(buffer, sizeof(buffer));
        }

        while (bytesRead < 8)
          buffer[bytesRead++] = 0xff;
          
        currentByte += bytesRead;
        crc = crc32_word(crc, *(uint32_t*)&buffer[0]);
        crc = crc32_word(crc, *(uint32_t*)&buffer[4]);      
      
        tx_frame.identifier = 0x7dd;
        tx_frame.data_length_code = 8;
        tx_frame.data[0] = buffer[0];
        tx_frame.data[1] = buffer[1];
        tx_frame.data[2] = buffer[2];
        tx_frame.data[3] = buffer[3];
        tx_frame.data[4] = buffer[4];
        tx_frame.data[5] = buffer[5];
        tx_frame.data[6] = buffer[6];
        tx_frame.data[7] = buffer[7];
        
        updstate = SEND_PAGE;
        twai_transmit(&tx_frame, pdMS_TO_TICKS(10));
      }
      else if (rxframe->data[0] == 'C') {
        tx_frame.identifier = 0x7dd;
        tx_frame.data_length_code = 4;
        tx_frame.data[0] = crc & 0xFF;
        tx_frame.data[1] = (crc >> 8) & 0xFF;
        tx_frame.data[2] = (crc >> 16) & 0xFF;
        tx_frame.data[3] = (crc >> 24) & 0xFF;

        updstate = CHECK_CRC;
        twai_transmit(&tx_frame, pdMS_TO_TICKS(10));
      }
      break;
    case CHECK_CRC:
      crc = 0xFFFFFFFF;
      DBG_OUTPUT_PORT.printf("Sent bytes %u-%u... ", currentPage * PAGE_SIZE_BYTES, currentByte);
      if (rxframe->data[0] == 'P') {
        updstate = SEND_PAGE;
        currentPage++;
        DBG_OUTPUT_PORT.printf("CRC Good\r\n");
        handleUpdate(rxframe);
      }
      else if (rxframe->data[0] == 'E') {
        updstate = SEND_PAGE;
        currentByte = currentPage * PAGE_SIZE_BYTES;
        DBG_OUTPUT_PORT.printf("CRC Error\r\n");
        handleUpdate(rxframe);
      }
      else if (rxframe->data[0] == 'D') {
        updstate = REQUEST_JSON;
        state = OBTAINSERIAL;
        retries = 50;
        updateFile.close();
        DBG_OUTPUT_PORT.printf("Done!\r\n");
      }
      break;
  }
  
}

int StartUpdate(String fileName) {
  updateFile = SPIFFS.open(fileName, "r");
  //Reset host processor
  setValueSdo(SDO_INDEX_COMMANDS, SDO_CMD_RESET, 1);
  updstate = SEND_MAGIC;
  
  return (updateFile.size() + PAGE_SIZE_BYTES - 1) / PAGE_SIZE_BYTES;
}

int GetCurrentUpdatePage() {
  return currentPage;
}

void SendJson(WiFiClient client) {
  if (state != IDLE) return;
  
  DynamicJsonDocument doc(30000);
  twai_message_t rxframe;
  
  File file = SPIFFS.open(jsonFileName, "r");
  deserializeJson(doc, file);
  file.close();

  JsonObject root = doc.as<JsonObject>();

  for (JsonPair kv : root) {
    int id = kv.value()["id"].as<int>();
    
    if (id > 0) {
      requestSdoElement(SDO_INDEX_PARAM_UID | (id >> 8), id & 0xff);
    
      if (twai_receive(&rxframe, pdMS_TO_TICKS(10)) == ESP_OK) {
        kv.value()["value"] = ((double)*(int32_t*)&rxframe.data[4]) / 32;
      }
    }
  }
  WriteBufferingStream bufferedWifiClient{client, 1000};
  serializeJson(doc, bufferedWifiClient);
}

SetResult SetValue(String name, double value) {
  if (state != IDLE) return CommError;
  
  twai_message_t rxframe;
  
  int id = getId(name);

  setValueSdo(SDO_INDEX_PARAM_UID | (id >> 8), id & 0xFF, value);

  if (twai_receive(&rxframe, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (rxframe.data[0] == SDO_RESPONSE_DOWNLOAD)
      return Ok;
    else if (*(uint32_t*)&rxframe.data[4] == SDO_ERR_RANGE)
      return ValueOutOfRange;
    else
      return UnknownIndex;
  }
  else {
    return CommError;
  }
}

String StreamValues(String names, int samples) {
  if (state != IDLE) return "";
  
  twai_message_t rxframe;

  int ids[30], numItems = 0;
  String result;
  
  for (int pos = 0; pos >= 0; pos = names.indexOf(',', pos + 1)) {
    String name = names.substring(pos + 1, names.indexOf(',', pos + 1));
    ids[numItems++] = getId(name);
  }

  for (int i = 0; i < samples; i++) {
    for (int item = 0; item < numItems; item++) {
      int id = ids[item];
      requestSdoElement(SDO_INDEX_PARAM_UID | (id >> 8), id & 0xFF);
    }
    
    int item = 0;
    while (twai_receive(&rxframe, pdMS_TO_TICKS(10)) == ESP_OK) {
      if (item > 0) result += ",";
      if (rxframe.data[0] == 0x80)
        result += "0";
      else {
        int receivedItem = (rxframe.data[1] << 8) + rxframe.data[3];
        
        if (receivedItem == ids[item])
          result += String(((double)*(uint32_t*)&rxframe.data[4]) / 32, 2);
        else
          result += "0";
      }
      item++;
    }
    result += "\r\n";
  }
  return result;
}

double GetValue(String name) {
  if (state != IDLE) return 0;
  
  twai_message_t rxframe;
  
  int id = getId(name);

  requestSdoElement(SDO_INDEX_PARAM_UID | (id >> 8), id & 0xFF);

  if (twai_receive(&rxframe, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (rxframe.data[0] == 0x80)
      return 0;
    else
      return ((double)*(uint32_t*)&rxframe.data[4]) / 32;
  }
  else {
    return 0;
  }
}

void Init(uint8_t nodeId) {
  twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = GPIO_NUM_25,
        .rx_io = GPIO_NUM_26,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 30,
        .rx_queue_len = 30,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0
  };

  uint16_t id = 0x580 + nodeId;
    
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = {.acceptance_code = (uint32_t)(id << 5) | (uint32_t)(0x7de << 21),
                                   .acceptance_mask = 0x001F001F,
                                   .single_filter = false};

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
     printf("Driver installed\n");
  } else {
     printf("Failed to install driver\n");
     return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    printf("Driver started\n");
  } else {
    printf("Failed to start driver\n");
    return;
  }

  _nodeId = nodeId;
  state = OBTAINSERIAL;
  requestSdoElement(SDO_INDEX_SERIAL, 0);
  DBG_OUTPUT_PORT.println("Initialized CAN");
}

void Loop() {
  bool recvdResponse = false;
  twai_message_t rxframe;
  
  if (twai_receive(&rxframe, 0) == ESP_OK) {
    if (rxframe.identifier == (0x580 | _nodeId)) {
      handleSdoResponse(&rxframe);
      recvdResponse = true;
    }
    else if (rxframe.identifier == 0x7de)
      handleUpdate(&rxframe);
    else
      DBG_OUTPUT_PORT.printf("Received unwanted frame %u\r\n", rxframe.identifier);
  }
  
  if (updstate == REQUEST_JSON) {
    //Re-download JSON if necessary
    
    retries--;
    
    if (recvdResponse || retries < 0) 
      updstate = UPD_IDLE; //if request was successful
    else
      requestSdoElement(SDO_INDEX_SERIAL, 0);
      
     delay(100);
  }

}

}
