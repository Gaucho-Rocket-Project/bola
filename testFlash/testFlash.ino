#include <LittleFS.h>

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  // Mount LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println(":x: LittleFS Mount Failed");
    while (1);
  }
  Serial.println(":white_check_mark: LittleFS Mounted Successfully");
  // List files
  // Serial.println(">> Listing files in root directory:");
  // File root = LittleFS.open("/");
  // File file = root.openNextFile();
  // while (file) {
  //   Serial.print("File: ");
  //   Serial.println(file.name());
  //   file = root.openNextFile();
  // }
  // Try to open the test.csv file
  File root = LittleFS.open("/");
  while (File file = root.openNextFile()) {
    Serial.printf("Found %s  (%u bytes)\n", file.name(), file.size());
    file.close();
  }
  File csvFile = LittleFS.open("/test.csv", FILE_READ);
  if (!csvFile) {
    Serial.println(":x: Failed to open /test.csv");
    return;
  }
  Serial.println(">> Contents of /test.csv:");
  while (csvFile.available()) {
    String line = csvFile.readStringUntil('\n');
    Serial.println(line);
  }
  csvFile.close();
}
void loop() {
  // nothing else to do
}