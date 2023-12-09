
# Installing the esp32-web-interface on ESP32

Goal: Use an ESP32 (ESP32 Dev Module, WROOM32) as web-interface with CAN connection.
References: https://github.com/jsphuebner/esp32-web-interface/tree/can-backend

## Preconditions:
- Arduino IDE 2.2.1 installed (quite new in December 2023).
- ESP32 support installed in the Arduino IDE.
- Hardware: ESP32 Dev Module. This contains a micro-USB for initial flashing.
- CAN transceiver added to the ESP32 module. CAN_TX = GPIO_NUM_25, CAN_RX = GPIO_NUM_26.

## Getting the project
```
    git clone git@github.com:jsphuebner/esp32-web-interface.git
    cd esp32-web-interface/
    git checkout can-backend
```

## Opening in Arduino IDE and compiling
- Double-click on the ino file, this will open the project in Arduino IDE.
- Connect the ESP32 with an micro-USB cable to the PC.
- In the Arduino IDE, select the Board (ESP32 Dev Module) and the port.
- CPU Freq = 240, Erase All Flash = OFF, PartitionScheme Default 4MB with SPIFF, PSRAM Disabled
- Try to upload the sketch to the ESP32. This may fail due to some missing libraries, you can
install them via the Library Manager.
- Finally the sketch is uploaded.
- When trying to access the web interface http://inverter.local/ it looks quite empty. This is because of missing web page files in the flash file system of the ESP32.

## Problem: How to upload the files to the ESP32?

For Arduino IDE 2.x, there is no 'Sketch Data Upload' plugin. In the old Arduino 1.x world, the plugin could be installed to easily upload files into the ESP's flash file system.

Workaround:
- https://github.com/palmerr23/ESP32-OTA-and-File-Manager Is a sketch which allows file system access (write, read, delete, list).
    - clone it
    - rename the folder, to match the folder name and ino file name (otherwise the Arduino IDE will complain)
    - enter your wifi credentials in the beginning of ESP32_OTA_FILESYS.ino
    - In the Arduino IDE, use the same settings as above:
        - Board ESP32 Dev Module
        - CPU Freq = 240, Erase All Flash = OFF, PartitionScheme Default 4MB with SPIFF, PSRAM Disabled
    - compile and upload the sketch to the ESP32
    - reset the ESP32. The ESP will connect to your wifi network.
    - In a browser, navigate to http://esp32ota.local/ This shows a page which offers OTA update and viewing/changing the file system content.
    - If it says "SPIFFS filesystem not found." press "Format FS".
    - If shows in the Log section "Formatted FS SPIFFS" and after reloading in the file system section "SPIFFS filesystem found."
    - In the section "upload files", select each of files in  ...\esp32-webinterface\esp32-web-interface\data and upload it.
    
- Now back in Arduino IDE, change to the esp32-web-interface project. 
- Compile and upload the sketch.
- In serial console, select 115kBaud. This shows
```
        - No RTC found, defaulting to sequential file names
        - E (161) sdmmc_common: sdmmc_init_ocr: send_op_cond (1) returned 0x107
        - E (162) vfs_fat_sdmmc: sdmmc_card_init failed (0x107).
        Couldn't start SD_MMC
        Driver installed
        Driver started
        Initialized CAN
````
- This looks normal, because we neigther connected an RTC nor an SD card.
- Search for a Wifi network "ESP32RETSSID" and connect to it.
- In browser, navigate to http://inverter.local/
- If you want to change files of the web interface, you can again flash ESP32_OTA_FILESYS.ino, and
  use this to add/change/delete file in the SPIFF. Afterwards, flash again the esp32-web-interface project.
- A shorter way to change files is, to use in http://inverter.local/ the "Files" tab in the left menu.
- An alternative solution for uploading the files using curl is explained here: https://openinverter.org/forum/viewtopic.php?p=3893#p3893
  (not tested).

- In http://inverter.local/ section "Wifi Settings" you can add the credentials of your router, so the web interface will connect
   at the next startup to your wifi network. You will still reach it under http://inverter.local/

- To connect to a certain control unit, select the node ID. (todo: where to find the node ID of a certain device?) For ccs32clara,
   it is set in main.cpp in this way: sdo.SetNodeId(22)
   
- When changing the NodeId in the web interface, the esp32-web-interface will send a CAN message, e.g. at ID 0x616 for nodeID 22. If the related control unit is on the bus, it will respond, e.g. on ID 0x596 for nodeID 22.

- The esp32-web-interface will automatically fetch the parameter descriptions (names, scalings etc) from the control unit.
