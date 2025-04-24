#include <Wire.h>
extern "C" {
  #include "bmp5.h"
}

#define I2C_ADDRESS 0x76  // change to 0x77 if your sensor uses that address

bmp5_dev bmp5;
bmp5_sensor_data sensor_data;
bmp5_osr_odr_press_config osr_odr_cfg;

int8_t read_bytes(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg_addr);
  if (Wire.endTransmission(false) != 0) return -1;

  Wire.requestFrom(I2C_ADDRESS, (uint8_t)len);
  for (uint32_t i = 0; i < len && Wire.available(); i++) {
    data[i] = Wire.read();
  }
  return 0;
}

int8_t write_bytes(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg_addr);
  for (uint32_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  return Wire.endTransmission() == 0 ? 0 : -1;
}

void delay_us(uint32_t period, void *intf_ptr) {
  delayMicroseconds(period);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();  // default ESP32 SDA=SDA, SCL=SCL (GPIO 21/22)

  bmp5.intf = BMP5_I2C_INTF;
  bmp5.read = read_bytes;
  bmp5.write = write_bytes;
  bmp5.delay_us = delay_us;
  bmp5.intf_ptr = nullptr;

  int8_t rslt = bmp5_init(&bmp5);
  if (rslt != 0) {
    Serial.println("BMP5 init failed");
    while (1);
  }

  bmp5_get_osr_odr_press_config(&osr_odr_cfg, &bmp5);
  osr_odr_cfg.press_en = BMP5_ENABLE;
  bmp5_set_osr_odr_press_config(&osr_odr_cfg, &bmp5);

  bmp5_set_power_mode(BMP5_POWERMODE_FORCED, &bmp5);
}

void loop() {
  bmp5_set_power_mode(BMP5_POWERMODE_FORCED, &bmp5); // trigger a single measurement
  bmp5_get_sensor_data(&sensor_data, &osr_odr_cfg, &bmp5);

  Serial.print("Temperature (°C): ");
  Serial.println(sensor_data.temperature);

  Serial.print("Pressure (Pa): ");
  Serial.println(sensor_data.pressure);

  delay(1000);  // wait 1 second between reads
}
