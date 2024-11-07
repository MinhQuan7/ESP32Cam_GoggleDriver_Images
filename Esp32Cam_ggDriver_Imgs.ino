//======================================== Including the libraries.
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Base64.h"
#include "esp_camera.h"
#include "SD_MMC.h"  // Thêm thư viện SD card
#include "time.h"    // Thêm thư viện thời gian để đặt tên file
#include <vector>
#include <algorithm>

//========================================
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;  // GMT+7 (Vietnam time: 7*3600)
const int daylightOffset_sec = 0;
//======================================== CAMERA_MODEL_AI_THINKER GPIO.
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
//========================================

// LED Flash PIN (GPIO 4)
#define FLASH_LED_PIN 4

//======================================== Enter your WiFi ssid and password.
const char* ssid = "eoh.io";
const char* password = "Eoh@2020";
//========================================

//======================================== Replace with your "Deployment ID" and Folder Name.
String myDeploymentID = "AKfycbyFIAeK-p3mZwWubDVvnW8-rltmF5I_2kGaDWuKtbiCbAU9uAlInAwIGd3TBYp4Lx2jCQ";
String myMainFolderName = "ESP32_CAM";
//========================================

//======================================== Variables for Timer/Millis.
unsigned long previousMillis = 0;
const int Interval = 20000;  //--> Capture and Send a photo every 20 seconds.
//========================================

// Variable to set capture photo with LED Flash.
// Set to "false", then the Flash LED will not light up when capturing a photo.
// Set to "true", then the Flash LED lights up when capturing a photo.
bool LED_Flash_ON = true;

// Initialize WiFiClientSecure.
WiFiClientSecure client;

//________________________________________________________________________________ Test_Con()
// This subroutine is to test the connection to "script.google.com".
void Test_Con() {
  const char* host = "script.google.com";
  while (1) {
    Serial.println("-----------");
    Serial.println("Connection Test...");
    Serial.println("Connect to " + String(host));

    client.setInsecure();

    if (client.connect(host, 443)) {
      Serial.println("Connection successful.");
      Serial.println("-----------");
      client.stop();
      break;
    } else {
      Serial.println("Connected to " + String(host) + " failed.");
      Serial.println("Wait a moment for reconnecting.");
      Serial.println("-----------");
      client.stop();
    }

    delay(1000);
  }
}


//=====================Hàm khởi tạo thời gian =========================
void initTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 5) {
    Serial.println("Failed to obtain time, retrying...");
    delay(1000);
    retry++;
  }

  if (retry >= 5) {
    Serial.println("Could not get time from NTP. Check your internet connection.");
  } else {
    Serial.println("Time synchronized successfully");
  }
}

//________________________________________________________________________________
//=======================SD Card=========================================
String getFileName() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return String("photo_") + String(millis()) + ".jpg";
  }
  char timeStringBuff[50];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y%m%d_%H%M%S", &timeinfo);
  return String("/photo_") + String(timeStringBuff) + ".jpg";
}

//=======================init SDCard Function==============================

esp_err_t res = ESP_OK;
// Function to initialize SD Card
static esp_err_t init_sdcard() {
  // Khởi tạo với one-bit mode
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Failed to mount SD card VFAT filesystem.");
    Serial.println("Please check if:");
    Serial.println("1. SD Card is properly inserted");
    Serial.println("2. SD Card pins are properly connected");
    return ESP_FAIL;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return ESP_FAIL;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // Thử tạo một file test để kiểm tra quyền ghi
  File testFile = SD_MMC.open("/test.txt", FILE_WRITE);
  if (!testFile) {
    Serial.println("Failed to create test file - Check SD card write permissions");
    return ESP_FAIL;
  }
  testFile.close();
  SD_MMC.remove("/test.txt");

  return ESP_OK;
}

//==========================end initSDCard===========================



//================ Function to save photo to SD Card===========
bool savePhotoToSD(camera_fb_t* fb) {
  String path = getFileName();
  Serial.printf("Picture file name: %s\n", path.c_str());

  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
    // Thử tạo thư mục nếu chưa tồn tại
    if (!SD_MMC.mkdir("/photos")) {
      Serial.println("Failed to create photos directory");
      return false;
    }
    // Thử mở file lại
    file = SD_MMC.open(path.c_str(), FILE_WRITE);
    if (!file) {
      return false;
    }
  }

  size_t written = file.write(fb->buf, fb->len);
  file.close();

  if (written != fb->len) {
    Serial.println("Failed to write complete file");
    return false;
  }

  Serial.printf("Saved file: %s, size: %u bytes\n", path.c_str(), fb->len);
  return true;
}
//====================end Save photo to SDCard=========================

//________________________________________________________________________________ SendCapturedPhotos()
// Subroutine for capturing and sending photos to Google Drive.
void SendCapturedPhotos() {
  const char* host = "script.google.com";
  Serial.println();
  Serial.println("-----------");
  Serial.println("Connect to " + String(host));

  client.setInsecure();

  //---------------------------------------- The Flash LED blinks once to indicate connection start.
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(100);
  digitalWrite(FLASH_LED_PIN, LOW);
  delay(100);
  //----------------------------------------

  //---------------------------------------- The process of connecting, capturing and sending photos to Google Drive.
  if (client.connect(host, 443)) {
    Serial.println("Connection successful.");

    if (LED_Flash_ON == true) {
      digitalWrite(FLASH_LED_PIN, HIGH);
      delay(100);
    }

    //.............................. Taking a photo.
    Serial.println();
    Serial.println("Taking a photo...");

    for (int i = 0; i <= 3; i++) {
      camera_fb_t* fb = NULL;
      fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        Serial.println("Restarting the ESP32 CAM.");
        delay(1000);
        ESP.restart();
        return;
      }
      esp_camera_fb_return(fb);
      delay(200);
    }

    camera_fb_t* fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
      return;
    }

    if (LED_Flash_ON == true) digitalWrite(FLASH_LED_PIN, LOW);

    Serial.println("Taking a photo was successful.");

    //===================================
    // Save to SD Card
    if (savePhotoToSD(fb)) {
      Serial.println("Saved photo to SD Card successfully");
    } else {
      Serial.println("Failed to save photo to SD Card");
    }

    //.............................. Sending image to Google Drive.
    Serial.println();
    Serial.println("Sending image to Google Drive.");
    Serial.println("Size: " + String(fb->len) + "byte");

    String url = "/macros/s/" + myDeploymentID + "/exec?folder=" + myMainFolderName;

    client.println("POST " + url + " HTTP/1.1");
    client.println("Host: " + String(host));
    client.println("Transfer-Encoding: chunked");
    client.println();

    int fbLen = fb->len;
    char* input = (char*)fb->buf;
    int chunkSize = 3 * 1000;  //--> must be multiple of 3.
    int chunkBase64Size = base64_enc_len(chunkSize);
    char output[chunkBase64Size + 1];

    Serial.println();
    int chunk = 0;
    for (int i = 0; i < fbLen; i += chunkSize) {
      int l = base64_encode(output, input, min(fbLen - i, chunkSize));
      client.print(l, HEX);
      client.print("\r\n");
      client.print(output);
      client.print("\r\n");
      delay(100);
      input += chunkSize;
      Serial.print(".");
      chunk++;
      if (chunk % 50 == 0) {
        Serial.println();
      }
    }
    client.print("0\r\n");
    client.print("\r\n");

    esp_camera_fb_return(fb);
    //..............................

    //.............................. Waiting for response.
    Serial.println("Waiting for response.");
    long int StartTime = millis();
    while (!client.available()) {
      Serial.print(".");
      delay(100);
      if ((StartTime + 10 * 1000) < millis()) {
        Serial.println();
        Serial.println("No response.");
        break;
      }
    }
    Serial.println();
    while (client.available()) {
      Serial.print(char(client.read()));
    }
    //..............................

    //.............................. Flash LED blinks once as an indicator of successfully sending photos to Google Drive.
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(500);
    //..............................
  } else {
    Serial.println("Connected to " + String(host) + " failed.");

    //.............................. Flash LED blinks twice as a failed connection indicator.
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(500);
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(500);
    //..............................
  }
  //----------------------------------------
  Serial.println("\n");
  Serial.println("**********************************");
  client.stop();
}

//__________________________________________________________________________________________________

//=============================Extract Video When press button=======================
// Struct để lưu thông tin file
#define BUTTON_PIN 12

struct FileInfo {
  String name;
  time_t modified;
};

// Struct để lưu thông tin file với các thành phần cơ bản
struct FileEntry {
    String path;
    unsigned long timestamp;
    
    FileEntry(String p, unsigned long t) : path(p), timestamp(t) {}
    
    // Operator so sánh cho việc sắp xếp
    bool operator < (const FileEntry& other) const {
        return timestamp > other.timestamp;  // Sắp xếp giảm dần (mới nhất trước)
    }
};


// Function để lấy 4 file ảnh mới nhất
std::vector<String> getLatestPhotos() {
    std::vector<FileEntry> fileList;
    std::vector<String> result;
    
    File root = SD_MMC.open("/");
    if(!root){
        Serial.println("Failed to open root directory");
        return result;
    }
    if(!root.isDirectory()){
        Serial.println("Root is not a directory");
        return result;
    }

    // Quét tất cả các file trong thư mục gốc
    File file = root.openNextFile();
    while(file) {
        if(!file.isDirectory()) {
            String fileName = String(file.name());
            if(fileName.endsWith(".jpg")) {
                // Lấy thời gian sửa đổi file
                struct stat st;
                String fullPath = "/" + fileName;
                if(stat(("/sdcard" + fullPath).c_str(), &st) == 0) {
                    fileList.push_back(FileEntry(fullPath, st.st_mtime));
                }
            }
        }
        file = root.openNextFile();
    }
    root.close();

    // Sắp xếp file theo thời gian
    std::sort(fileList.begin(), fileList.end());

    // Lấy 4 file mới nhất
    size_t count = min(size_t(4), fileList.size());
    for(size_t i = 0; i < count; i++) {
        result.push_back(fileList[i].path);
        Serial.println("Selected file: " + fileList[i].path);
    }

    return result;
}


// Function để tạo folder mới trên Google Drive
String createDriveFolder() {
    const char* host = "script.google.com";
    String folderName = "ESP32_Photos_" + String(time(nullptr));

    Serial.println();
    Serial.println("Creating folder in Google Drive.");

    if (client.connect(host, 443)) {
        String url = "/macros/s/" + myDeploymentID + "/exec?folder=" + folderName;

        client.println("POST " + url + " HTTP/1.1");
        client.println("Host: " + String(host));
        client.println("Transfer-Encoding: chunked");
        client.println("Connection: close");
        client.println();

        client.print("0\r\n");
        client.print("\r\n");

        Serial.println("Waiting for response.");
        long int startTime = millis();
        while (!client.available()) {
            Serial.print(".");
            delay(100);
            if ((startTime + 10 * 1000) < millis()) {
                Serial.println();
                Serial.println("No response.");
                client.stop();
                return "";
            }
        }
        Serial.println();

        String response = "";
        while (client.available()) {
            response += char(client.read());
        }
        Serial.println("Folder creation response: " + response);

        // Kiểm tra xem folder có được tạo thành công không
        if (response.indexOf("folderId") >= 0) {
            Serial.println("Folder created successfully: " + folderName);
            return folderName;
        } else {
            Serial.println("Folder creation failed.");
            return "";
        }

    } else {
        Serial.println("Connection to " + String(host) + " failed.");

        // Flash LED twice as an indicator of failed connection
        digitalWrite(FLASH_LED_PIN, HIGH);
        delay(500);
        digitalWrite(FLASH_LED_PIN, LOW);
        delay(500);
        digitalWrite(FLASH_LED_PIN, HIGH);
        delay(500);
        digitalWrite(FLASH_LED_PIN, LOW);
        delay(500);

        return "";
    }
    client.stop();
}

// Function to upload a file to Google Drive
bool uploadToGDrive(const String& fileName, const String& folderName) {
    const char* host = "script.google.com";
    bool success = false;
    
    Serial.println("Starting upload for: " + fileName);
    
    File file = SD_MMC.open(fileName, FILE_READ);
    if(!file) {
        Serial.println("Failed to open file for reading: " + fileName);
        return false;
    }
    
    if (!client.connect(host, 443)) {
        Serial.println("Connection failed");
        file.close();
        return false;
    }
    
    Serial.println("Uploading to folder: " + folderName);
    
    String url = "/macros/s/" + myDeploymentID + "/exec?folder=" + folderName;
    
    client.println("POST " + url + " HTTP/1.1");
    client.println("Host: " + String(host));
    client.println("Transfer-Encoding: chunked");
    client.println();
    
    const size_t bufferSize = 3000;
    uint8_t *buffer = (uint8_t*)malloc(bufferSize);
    if (!buffer) {
        Serial.println("Failed to allocate buffer");
        file.close();
        client.stop();
        return false;
    }
    
    while(file.available()) {
        size_t bytesRead = file.read(buffer, bufferSize);
        if(bytesRead > 0) {
            size_t outputLen = base64_enc_len(bytesRead);
            char* output = (char*)malloc(outputLen + 1);
            if (!output) {
                Serial.println("Failed to allocate output buffer");
                break;
            }
            
            base64_encode(output, (char*)buffer, bytesRead);
            
            client.printf("%X\r\n", strlen(output));
            client.print(output);
            client.print("\r\n");
            
            free(output);
        }
    }
    
    free(buffer);
    client.print("0\r\n\r\n");
    
    // Đợi phản hồi
    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 10000) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            if (line.indexOf("success") >= 0) {
                success = true;
            }
        }
    }
    
    file.close();
    client.stop();
    
    if (success) {
        Serial.println("Upload successful: " + fileName);
    } else {
        Serial.println("Upload may have failed: " + fileName);
    }
    
    return success;
}


// Function chính để xử lý extract và upload

// Main function to handle extract and upload process
void handleExtractAndUpload() {
    Serial.println("\nStarting extract and upload process...");
    
    // Đảm bảo SD card được mount
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("SD Card Mount Failed");
        return;
    }
    
    // Lấy danh sách file mới nhất
    std::vector<String> latestFiles = getLatestPhotos();
    if (latestFiles.empty()) {
        Serial.println("No photos found on SD Card");
        return;
    }
    
    Serial.printf("Found %d photos to upload\n", latestFiles.size());
    
    // Tạo folder mới
    String folderName = createDriveFolder();
    if (folderName.isEmpty()) {
        Serial.println("Failed to create folder name");
        return;
    }
    
    // Upload từng file
    int successCount = 0;
    for (const String& filePath : latestFiles) {
        Serial.printf("\nUploading file %d/%d: %s\n", 
                     successCount + 1, latestFiles.size(), filePath.c_str());
                     
        if (uploadToGDrive(filePath, folderName)) {
            successCount++;
            // Flash LED để báo hiệu upload thành công
            digitalWrite(FLASH_LED_PIN, HIGH);
            delay(200);
            digitalWrite(FLASH_LED_PIN, LOW);
            delay(200);
        }
        
        // Delay nhỏ giữa các lần upload
        delay(1000);
    }
    
    // Báo hiệu hoàn thành
    Serial.printf("\nUpload completed. %d/%d files uploaded successfully\n", 
                 successCount, latestFiles.size());
                 
    // Flash LED pattern để báo hiệu hoàn thành
    for(int i = 0; i < 3; i++) {
        digitalWrite(FLASH_LED_PIN, HIGH);
        delay(500);
        digitalWrite(FLASH_LED_PIN, LOW);
        delay(500);
    }
}


//==============================END EXTRACT VIDEOS WHEN PRESS BUTTON=============================
//________________________________________________________________________________ VOID SETUP()
void setup() {
  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println();
  delay(1000);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FLASH_LED_PIN, OUTPUT);

  // Setting the ESP32 WiFi to station mode.
  Serial.println();
  Serial.println("Setting the ESP32 WiFi to station mode.");
  WiFi.mode(WIFI_STA);

  //---------------------------------------- The process of connecting ESP32 CAM with WiFi Hotspot / WiFi Router.
  Serial.println();
  Serial.print("Connecting to : ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  Serial.println("============================");
  Serial.println("Initializing time...");
  initTime();
  // SD camera init
  Serial.println("Mounting the SD card ...");
  esp_err_t card_err = init_sdcard();
  if (card_err != ESP_OK) {
    Serial.printf("SD Card init failed with error 0x%x", card_err);
    return;
  }

  Serial.println("Initializing SD Card...");
  if (init_sdcard() != ESP_OK) {
    Serial.println("SD Card initialization failed! - - - - -- Failed  - - - Failed");
    // ESP.restart();
  }
  // The process timeout of connecting ESP32 CAM with WiFi Hotspot / WiFi Router is 20 seconds.
  // If within 20 seconds the ESP32 CAM has not been successfully connected to WiFi, the ESP32 CAM will restart.
  // I made this condition because on my ESP32-CAM, there are times when it seems like it can't connect to WiFi, so it needs to be restarted to be able to connect to WiFi.
  int connecting_process_timed_out = 20;  //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(250);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(250);
    if (connecting_process_timed_out > 0) connecting_process_timed_out--;
    if (connecting_process_timed_out == 0) {
      Serial.println();
      Serial.print("Failed to connect to ");
      Serial.println(ssid);
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
    }
  }

  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.println();
  Serial.print("Successfully connected to ");
  Serial.println(ssid);
  //Serial.print("ESP32-CAM IP Address: ");
  //Serial.println(WiFi.localIP());
  //----------------------------------------

  //---------------------------------------- Set the camera ESP32 CAM.
  Serial.println();
  Serial.println("Set the camera ESP32 CAM...");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 8;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println();
    Serial.println("Restarting the ESP32 CAM.");
    delay(1000);
    ESP.restart();
  }

  sensor_t* s = esp_camera_sensor_get();

  // Selectable camera resolution details :
  // -UXGA   = 1600 x 1200 pixels
  // -SXGA   = 1280 x 1024 pixels
  // -XGA    = 1024 x 768  pixels
  // -SVGA   = 800 x 600   pixels
  // -VGA    = 640 x 480   pixels
  // -CIF    = 352 x 288   pixels
  // -QVGA   = 320 x 240   pixels
  // -HQVGA  = 240 x 160   pixels
  // -QQVGA  = 160 x 120   pixels
  s->set_framesize(s, FRAMESIZE_SXGA);  //--> UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

  Serial.println("Setting the camera successfully.");
  Serial.println();

  delay(1000);

  Test_Con();

  Serial.println();
  Serial.println("ESP32-CAM captures and sends photos to the server every 20 seconds.");
  Serial.println();
  delay(2000);
}
//________________________________________________________________________________

//=============================VOID LOOP========================
void loop() {
    static bool buttonPressed = false;
    
    // Xử lý nút nhấn với debounce
    if (digitalRead(BUTTON_PIN) == 0) {  // Nút được nhấn
        if (!buttonPressed) {  // Chỉ xử lý khi nút mới được nhấn
            delay(50);  // Debounce
            if (digitalRead(BUTTON_PIN) == 0) {
                Serial.println("\nButton pressed - Starting extract and upload...");
                buttonPressed = true;
                handleExtractAndUpload();
            }
        }
    } else {
        buttonPressed = false;  // Reset trạng thái nút nhấn
    }
    
    // Chức năng chụp và upload tự động vẫn hoạt động
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= Interval) {
        previousMillis = currentMillis;
        SendCapturedPhotos();
    }
}