#  ESP32 Sensor Code

This repository contains the setup and code for using an ESP32 with sensors (IMU and barometer) for the Gaucho Rocket Project.

>  *The ESP32 uses a USB-C connection for uploading code.*

---

##  Requirements

###  Arduino IDE Setup

- Make sure you're using the **Arduino IDE**
- Select the following under **Tools**:
  - **Board:** `ESP32 Dev Module`
  - **Port:** The correct COM port your ESP32 is connected to

---

###  Libraries to Install

Install the following libraries via the **Arduino Library Manager**, or manually download them from GitHub and include them:

- **Madgwick** by Arduino  
- **Mahony** by Arduino  
- **Adafruit BMP3XX** by Adafruit  
- **Adafruit BusIO** by Adafruit  
- **Adafruit Unified Sensor** by Adafruit  
- **ESP32SPISlave** by hideakitai  
- **ICM20948_WE** by Wolfgang Ewald  
- **SparkFun BMP581 Arduino Library** by SparkFun Electronics  
- **SparkFun 9DoF IMU Breakout - ICM 20948** by SparkFun Electronics  

> 💡 **Windows users**: Install the [CP210x Universal Windows Driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) if your board doesn’t appear under **Ports**.

---

##  IMU-20948 and DMP Setup (for IMU)

1. Navigate to:  
   `Documents/Arduino/libraries/SparkFun_ICM-20948_ArduinoLibrary/src/util/`

2. Open `ICM_20948_C.h` and uncomment the following line:   `#define ICM_20948_USE_DMP`
3. Wire the ICM-20948 to the ESP32 using this schematic: <p align="center"> <img src="[https://github.com/user-attachments/assets/26625338-ee19-4321-a04d-c90d5f5c1a84](https://github.com/user-attachments/assets/55271a32-4c1b-4f5f-b4ea-8b9dc3966151)" alt="IMU wiring schematic"/> </p>
4. Open or paste the provided sketch from this repository into a new Arduino IDE project.
5. Upload the sketch, and hold the boot button on the ESP32 while uploading
6. Open Serial Moniter at `115200 baud` and when prompted enter any key
