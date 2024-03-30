/*
  The TFT_eSPI library incorporates an Adafruit_GFX compatible
  button handling class, this sketch is based on the Arduin-o-phone
  example.

  PIN Connection with ESP 32 Wroom - Dev module

  ESP32 -> TFT PIN
  3v3 -> Vcc
  gnd -> gnd
  D15 -> CS
  D4 -> Reset
  D27 -> DC
  D23 -> MOSI
  D18 -> SCK
  3v3 -> LED (This can be connected with any other GPIO pin to switch on and off)
  D19 -> MISO
  ******* Touch PIN *******
  D18 -> T_CLK
  D21 -> T_CS
  D23(MOSI) -> T_DIN
  D19(MISO) -> T_DO
  D35 -> T_IRQ

  ********************
   To do https://github.com/Bodmer/TJpg_Decoder/issues/45
   Large image crashing decoding
  *******************
*/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <vector>

#include "FS.h"
#include "SPIFFS.h" // Required for ESP32 only
#include <SPI.h>
#include <TFT_eSPI.h>      // Hardware-specific library
#include "List_SPIFFS.h"
// Include the jpeg decoder library
#include <TJpg_Decoder.h>

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

#define WIFI_ANALYSER

#ifdef WIFI_ANALYSER
  // Graph constant
  #define WIDTH 320
  #define HEIGHT 240
  #define BANNER_HEIGHT 16
  #define GRAPH_BASELINE (HEIGHT - 18)
  #define GRAPH_HEIGHT (HEIGHT - 52)
  #define CHANNEL_WIDTH (WIDTH / 16)

  // RSSI RANGE
  #define RSSI_CEILING -40
  #define RSSI_FLOOR -100
  #define NEAR_CHANNEL_RSSI_ALLOW -70
  // Channel color mapping from channel 1 to 14
  uint16_t channel_color[] = {
    TFT_RED, TFT_ORANGE, TFT_YELLOW, TFT_GREEN, TFT_CYAN, TFT_MAGENTA,
    TFT_RED, TFT_ORANGE, TFT_YELLOW, TFT_GREEN, TFT_CYAN, TFT_MAGENTA,
    TFT_RED, TFT_ORANGE
  };

  uint8_t scan_count = 0;
#endif

#define CHUNK_LENGTH 245
std::vector<uint8_t> byte_vector;
uint32_t fpsLastTime = 0;
int nbFrames = 0;
String size="";
int rssi_display=0;
int prev_rssi_display=0;

typedef struct {
  unsigned frame_ctrl: 16;
  unsigned duration_id: 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl: 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xE4, 0x65, 0xB8, 0x20, 0x78, 0x0C}; //esp32 dev board MAC address for receiving E4:65:B8:20:78:0C
String broadcastDevice = "E4:65:B8:20:78:0C";


// This next function will be called during decoding of the jpeg file to render each
// 16x16 or 8x8 image tile (Minimum Coding Unit) to the TFT.
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;

  // STM32F767 processor takes 43ms just to decode (and not draw) jpeg (-Os compile option)
  // Total time to decode and also draw to TFT:
  // SPI 54MHz=71ms, with DMA 50ms, 71-43 = 28ms spent drawing, so DMA is complete before next MCU block is ready
  // Apparent performance benefit of DMA = 71/50 = 42%, 50 - 43 = 7ms lost elsewhere
  // SPI 27MHz=95ms, with DMA 52ms. 95-43 = 52ms spent drawing, so DMA is *just* complete before next MCU block is ready!
  // Apparent performance benefit of DMA = 95/52 = 83%, 52 - 43 = 9ms lost elsewhere
#ifdef USE_DMA
  // Double buffering is used, the bitmap is copied to the buffer by pushImageDMA() the
  // bitmap can then be updated by the jpeg decoder while DMA is in progress
  if (dmaBufferSel) dmaBufferPtr = dmaBuffer2;
  else dmaBufferPtr = dmaBuffer1;
  dmaBufferSel = !dmaBufferSel; // Toggle buffer selection
  //  pushImageDMA() will clip the image block at screen boundaries before initiating DMA
  tft.pushImageDMA(x, y, w, h, bitmap, dmaBufferPtr); // Initiate DMA - blocking only if last DMA is not complete
  // The DMA transfer of image block to the TFT is now in progress...
#else
  // Non-DMA blocking alternative
  tft.pushImage(x, y, w, h, bitmap);  // Blocking, so only returns when image block is drawn
#endif
  // Return 1 to decode next block.
  return 1;
}

void display_signal(boolean skip_draw=true){
  // Set "cursor" at top left corner of display (0,0) and select font 2
        // (cursor will move to next line automatically during printing with 'tft.println'
        //  or stay on the line is there is room for the text with tft.print)
          
          if (prev_rssi_display != rssi_display &&  skip_draw) {
            //First remove the previous text
            tft.setCursor(0, 0, 2);
            tft.setRotation(2);
            tft.setTextColor(TFT_BLACK);  
            tft.setTextSize(2);
            tft.println(prev_rssi_display);   

            //write new text     
          // Set the font colour to be white with a black background, set text size multiplier to 1
            tft.setCursor(0, 0, 2);
            tft.setRotation(2);
            tft.setTextColor(TFT_WHITE);  
            tft.setTextSize(2);
            // We can now plot text on screen using the "print" class
            
            tft.println(rssi_display);
            prev_rssi_display = rssi_display;
            //delay(100);
          } else {

            //write new text     
          // Set the font colour to be white with a black background, set text size multiplier to 1
            tft.setCursor(0, 0, 2);
            tft.setRotation(2);
            tft.setTextColor(TFT_WHITE);  
            tft.setTextSize(2);
            // We can now plot text on screen using the "print" class
            
            tft.println(rssi_display);
            prev_rssi_display = rssi_display;

          }
}

//call back to measure WiFiNow signal
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  int rssi = ppkt->rx_ctrl.rssi;
  rssi_display = rssi;
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  //memcpy(&incomingReadings, incomingData, len);
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  //Serial.printf("packet Length: %d\n", len);
   display_signal(true);

  std::vector<uint8_t> temp_vector(&incomingData[0],&incomingData[len]);

  //Serial.printf("incomingData[0]: %u \n",incomingData[0]);
  //Serial.printf("temp_vector Length: %u\n", temp_vector.size());

      if (len == CHUNK_LENGTH && 
      incomingData[0] == 255 && 
      incomingData[1] == 216 &&
      incomingData[2] == 255) { // FF D8 FF
        byte_vector.clear();
        
      }

  byte_vector.insert(byte_vector.end(), temp_vector.begin(), temp_vector.end());

   fs::File f = SPIFFS.open("/tmp.jpg", FILE_APPEND,true);
        if (!f) {
        Serial.println("file open failed");
        return;
         } else {
          f.write(incomingData,len);  
          f.close();
         }
  //Serial.print("byte_vector Length: ");
  //Serial.println(byte_vector.size());

      if (len != CHUNK_LENGTH && 
      incomingData[len - 2] == 255 && 
      incomingData[len - 1] == 217) { // FF D9


        size = byte_vector.size();
        Serial.print("Image size: ");
        Serial.println(size);

      uint32_t t = millis(); 
        //uint8_t* jpgData = byte_vector.data();

        tft.setRotation(7); //0-3 normal rotation 4-7 rotation+mirror image
        if (TJpgDec.drawFsJpg(0, 0, "/tmp.jpg")!= JDR_OK) {
             Serial.println("Decoding failure !");
             //SPIFFS.remove("/M81.jpg");
          }
        SPIFFS.remove("/tmp.jpg");

         display_signal(false);

        // Create buffer for read
        //byte newImage[byte_vector.size()];
        //std::copy(byte_vector.begin(), byte_vector.end(), newImage);
        
        // Time recorded for test purposes
               
          // How much time did rendering take (ESP8266 80MHz 262ms, 160MHz 149ms, ESP32 SPI 111ms, 8bit parallel 90ms
          t = millis() - t;
          Serial.print(t); Serial.println(" ms");
      
        
      }

     
  
}

//------------------------------------------------------------------------------------------

void setup() {
  
  //Create task decodeJpg to run on processor 0 to decode a Jpeg
  //xTaskCreatePinnedToCore(decodeJpg, "decodeJpg", 10000, NULL, 0, NULL, 0);

  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.enableLongRange(true);
  Serial.println(WiFi.macAddress());
  
  WiFi.disconnect(); 

  

// Initialise SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS initialisation failed!");
    //while (1) yield(); // Stay here twiddling thumbs waiting
  } else {
  Serial.println("\r\n SPIFFS Initialisation done.");
  // This is for demoe purposes only so that file is fetched each time this is run
  if (SPIFFS.exists("/tmp.jpg") == true) {
    //Serial.println("For test only, removing file");
    //SPIFFS.remove("/M81.jpg");
    SPIFFS.remove("/tmp.jpg");
    //SPIFFS.remove("/TouchCalData1");
  }
  }

  // Initialise the TFT
  tft.begin();
  
  #ifdef WIFI_ANALYSER

  tft.setRotation(3);
    tft.setTextSize(2);
    tft.fillScreen(TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setCursor(0, 0);
    tft.print(" ESP ");
    tft.setTextColor(TFT_WHITE, TFT_ORANGE);
    tft.print(" 32 ");
    tft.setTextColor(TFT_WHITE, TFT_GREEN);
    tft.print(" WiFi ");
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.print(" Analyzer");
  
  #else

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } 

  if (broadcastDevice == WiFi.macAddress().c_str()){
  // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);

  }

  tft.setTextColor(0xFFFF, 0x0000);
  tft.fillScreen(TFT_BLACK);

  // The jpeg image can be scaled by a factor of 1, 2, 4, or 8
  TJpgDec.setJpgScale(1);

  // The byte order can be swapped (set true for TFT_eSPI)
  TJpgDec.setSwapBytes(true);

  // The decoder must be given the exact name of the rendering function above
  TJpgDec.setCallback(tft_output);

  #endif

// rest for WiFi routine?
  delay(100);

}

//------------------------------------------------------------------------------------------

void loop(void) {
  delay(10000);
// List files stored in SPIFFS
//  listSPIFFS();
#ifdef WIFI_ANALYSER
uint8_t ap_count[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int32_t max_rssi[] = {-100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();

  // clear old graph
  tft.fillRect(0, BANNER_HEIGHT, 320, 224, TFT_BLACK);
  tft.setTextSize(1);

  if (n == 0) {
    tft.setTextColor(TFT_BLACK);
    tft.setCursor(0, BANNER_HEIGHT);
    tft.println("no networks found");
  } else {
    // plot found WiFi info
    for (int i = 0; i < n; i++) {
      int32_t channel = WiFi.channel(i);
      int32_t rssi = WiFi.RSSI(i);
      uint16_t color = channel_color[channel - 1];
      int height = constrain(map(rssi, RSSI_FLOOR, RSSI_CEILING, 1, GRAPH_HEIGHT), 1, GRAPH_HEIGHT);

      // channel stat
      ap_count[channel - 1]++;
      if (rssi > max_rssi[channel - 1]) {
        max_rssi[channel - 1] = rssi;
      }

      tft.drawLine(channel * CHANNEL_WIDTH, GRAPH_BASELINE - height, (channel - 1) * CHANNEL_WIDTH, GRAPH_BASELINE + 1, color);
      tft.drawLine(channel * CHANNEL_WIDTH, GRAPH_BASELINE - height, (channel + 1) * CHANNEL_WIDTH, GRAPH_BASELINE + 1, color);

      // Print SSID, signal strengh and if not encrypted
      tft.setTextColor(color);
      tft.setCursor((channel - 1) * CHANNEL_WIDTH, GRAPH_BASELINE - 15 - height);
      //Serial.print((channel - 1) * CHANNEL_WIDTH);
      //Serial.print(" , ");
      //Serial.print(GRAPH_BASELINE);
      //Serial.print(" , ");
      //Serial.println(height);
      tft.print(String(WiFi.SSID(i)).substring(0,6));
      tft.print('(');
      tft.print(rssi);
      tft.print(')');
      if (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) {
        tft.print('*');
      }

      // rest for WiFi routine?
      delay(10);
    }
  }

  // print WiFi stat
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, BANNER_HEIGHT);
  tft.print(n);
  tft.print(" networks found, suggested channels: ");
  bool listed_first_channel = false;
  for (int i = 1; i <= 11; i++) { // channels 12-14 may not available
    if ((i == 1) || (max_rssi[i - 2] < NEAR_CHANNEL_RSSI_ALLOW)) { // check previous channel signal strengh
      if ((i == sizeof(channel_color)) || (max_rssi[i] < NEAR_CHANNEL_RSSI_ALLOW)) { // check next channel signal strengh
        if (ap_count[i - 1] == 0) { // check no AP exists in same channel
          if (!listed_first_channel) {
            listed_first_channel = true;
          } else {
            tft.print(", ");
          }
          tft.print(i);
        }
      }
    }
  }

  // draw graph base axle
  tft.drawFastHLine(0, GRAPH_BASELINE, 320, TFT_WHITE);
  for (int i = 1; i <= 14; i++) {
    tft.setTextColor(channel_color[i - 1]);
    tft.setCursor((i * CHANNEL_WIDTH) - ((i < 10)?3:6), GRAPH_BASELINE + 2);
    tft.print(i);
    if (ap_count[i - 1] > 0) {
      tft.setCursor((i * CHANNEL_WIDTH) - ((ap_count[i - 1] < 10)?9:12), GRAPH_BASELINE + 11);
      tft.print('(');
      tft.print(ap_count[i - 1]);
      tft.print(')');
    }
  }


 #endif
   
}
