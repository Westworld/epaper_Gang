/*

Nodemcu ESP 8266 mit 4.2 inch Waveshare SPI e-paper
für Gang Stromanzeige

 * 4.2inch Display wemos mini
 * // BUSY -> D2, RST -> D4, DC -> D3, CS -> D8, CLK -> D5, DIN -> D7, GND -> GND, 3.3V -> 3.3V   +++
BUSY  D2 gpIO4
RST   D4 GPIO2
DC    D3 GPIO0
CS    D8 gpio15 -  pulldown 4.7k auf Masse !!!  -- bei Gang2 auf D1
CLK   D5-GPIO14
DIN   D7-GPIO13
GND   GND
3.3V  3V3

Dallas Temp auf D1  gpIO5
Bewegungssensor auf A0
DHT22 auf RX GPI03

D1-GPIO5  Trig als Output?  D0 geht nicht...
D6 gpiO12  Echo frei verwendbar

alternativ US-100
D1-GPIO5  US100_TX
D6 gpiO12  US100_RX kein temp Sensor


für zweites Display ohne Temp und ohne Bewegungssensor
CS auf D1
#define Gang 1
oder
#define Gang2 1
oder Gang3 1  // für gang mit temp und ultraschall HC-SR204

 */

#define Gang2 1
#define US100 1
// to switch from trigger to serial
#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoOTA.h>
#include <Console.h>


#include "main.h"


#ifdef Gang
GxEPD2_BW<GxEPD2_420, GxEPD2_420::HEIGHT> display(GxEPD2_420(/*CS=D8*/ D8, /*DC=D3*/ D3, /*RST=D4*/ D4, /*BUSY=D2*/ D2));

const char* wifihostname = "ESP_Epaper";
const char* jobname = "GangBitmap";
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS D1
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
#endif

#ifdef Gang2
GxEPD2_BW<GxEPD2_420, GxEPD2_420::HEIGHT> display(GxEPD2_420(/*CS=D8*/ D1, /*DC=D3*/ D3, /*RST=D4*/ D4, /*BUSY=D2*/ D2));
const char* wifihostname = "ESP_Epaper2";
const char* jobname = "epaper";

#endif

#ifdef Gang3
GxEPD2_BW<GxEPD2_420, GxEPD2_420::HEIGHT> display(GxEPD2_420(/*CS=D8*/ D8, /*DC=D3*/ D3, /*RST=D4*/ D4, /*BUSY=D2*/ D2));

const char* wifihostname = "ESP_Epaper3";
const char* jobname = "GangBitmap";
#ifndef US100
#include "DHT.h"
#define DHTPIN RX     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
const int trigPin = D1; //GPIO5;
const int echoPin = D6; // GPIO12;
#else
#include <SoftwareSerial.h>
#include <PingSerial.h>

const int US100_TX = D1;
const int US100_RX = D6;

SoftwareSerial US100Serial(US100_RX, US100_TX);
PingSerial us100(US100Serial, 10, 10000);  // Valid measurements are 650-1200mm

bool ping_enabled = TRUE;
unsigned int pingSpeed = 100; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer = 0;     // Holds the next ping time.

bool temp_enabled = TRUE;
unsigned int tempSpeed = 3000;
unsigned long tempTimer = 0;

float lastTemp=0;
#endif


#endif

const int httpPort  = 8000;

uint8_t bmpbuffer[16000];  // Bildbuffer

short RedrawCounter = 0;



bool Bewegung=false;
float LastDistance=0;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
delay(1000);
  display.init();// 115200); //  115200 for diagnostic output

    display.setRotation(1);
    helloWorld("Starte Network");


  Serial.println("setup done");

    Console::info("Connecting to SSID %s", WIFI_SSID);
    WiFi.hostname(wifihostname);  
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    short counter = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      if (++counter > 50)
        ESP.restart();
    }

    Console::info("WiFi connected");
    IPAddress ip = WiFi.localIP();
    Console::fatal("IP Address: %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]); 
    helloWorld("WLAN connected");
    ArduinoOTA.setHostname(wifihostname);  
    
  ArduinoOTA.onStart([]() {
    if (ArduinoOTA.getCommand() == U_FLASH) {
      helloWorld("Uploading Firmware");
      Console::info("Start updating sketch");
    } else { // U_SPIFFS
      Console::info("Start updating filesystem - unsupported?");


    }  
  });
  ArduinoOTA.onEnd([]() {
    Console::info("End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Console::info("Progress: %d", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Console::error("[%d]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Console::error("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Console::error("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Console::error("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Console::error("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Console::error("End Failed");
    }
  });


    ArduinoOTA.begin();

    #ifdef Gang
    sensors.begin();
    pinMode(A0, INPUT);
    #endif
    #ifdef Gang3
    #ifndef US100
    dht.begin();
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);   
    #else
      us100.begin();
    #endif
    #endif

}

void loop()
{
  showImage();

  myDelay(55000);
}

void showImage() {
      char logString[64];
    #ifdef Gang
      sensors.requestTemperatures();
      float temp = sensors.getTempCByIndex(0);
      Serial.print("temp: ");
      Serial.println(temp);
      sprintf(logString,"Strom?Job=%s&temp=%f", jobname, temp);
  #endif
  #ifdef Gang3
    #ifndef US100
        float h = dht.readHumidity();
      // Read temperature as Celsius (the default)
        float t = dht.readTemperature();
        sprintf(logString,"Strom?Job=%s&temp=%f&hum=%f&distance=%f", jobname, t, h, LastDistance);
        // sprintf(logString,"Strom?Job=%s&distance=%f", jobname, LastDistance);
    #else
        sprintf(logString,"Strom?Job=%s&temp=%f&distance=%f", jobname, lastTemp, LastDistance);
    #endif    
  #endif
  #ifdef Gang2
      sprintf(logString,"Strom?Job=%s", jobname);    
  #endif

    
  showBitmapBufferFrom_HTTP("192.168.0.34", "/4DAction/", logString, 0,0, false);
SendMessage(1);
}



void myDelay(long thedelay) {
  unsigned long start = millis();
  float raw;
  bool BewegAktiv=false;


  while ((start+thedelay)>millis()) {

    ArduinoOTA.handle();
    delay(1);

    #ifdef Gang
      raw = analogRead(A0);
      BewegAktiv=false;
      if (raw > 700)
        BewegAktiv=true;
      if (BewegAktiv != Bewegung) {
          Bewegung = BewegAktiv;
          showPartialUpdate(Bewegung);
          ReportBewegung(Bewegung, 0);
      }  
    #endif
    #ifdef Gang3
    #ifndef US100
    raw = GetDistance();
    LastDistance = raw;

    if (raw > 5)  {
      if (raw <80 )
        BewegAktiv=true;
      else
        BewegAktiv=false;

       char error[50];
       sprintf(error, "%d", (long) raw);
       ShowError(error);

      if (BewegAktiv != Bewegung) {
          //Serial.println(raw);
          Bewegung = BewegAktiv;
          showPartialUpdate(Bewegung);
          //ReportBewegung(Bewegung, (long) raw);
      }  
    }
    #else      
      byte data_available;
      byte msg;

      data_available = us100.data_available();

      if (data_available & DISTANCE) {

          float CurDistance=us100.get_distance() / 10;
          if (LastDistance != CurDistance) {
            Serial.print("Distance: ");
            Serial.println(CurDistance);
            LastDistance = CurDistance;
          }
          // Auswertung Bewegung
              if (LastDistance <70 )
                BewegAktiv=true;
              else
                BewegAktiv=false;

              //char error[50];
              //sprintf(error, "%d", (long) LastDistance);
              //ShowError(error);

              if (BewegAktiv != Bewegung) {
                //Serial.println(raw);
                Bewegung = BewegAktiv;
                //showPartialUpdate(Bewegung);
                // ReportBewegung(Bewegung, (long) LastDistance);
              }  
      }
      if (data_available & TEMPERATURE) {
          //Serial.print("Temperature: ");
          lastTemp=us100.get_temperature();
          //Serial.println(lastTemp);
      }

      if (ping_enabled && (millis() >= pingTimer)) {   // pingSpeed milliseconds since last ping, do another ping.
          pingTimer = millis() + pingSpeed;      // Set the next ping time.
          us100.request_distance();
      }

      if (temp_enabled && (millis() >= tempTimer)) {
          tempTimer = millis() + tempSpeed;
          us100.request_temperature();
      }

    #endif
    #endif    
    if (start > millis()) {
      start = 0;  // overflow
    }
  }

}


float GetDistance() {
  float duration, cm, average, result;
  short counter = 0;
  const short samples = 30;
  float collection[samples];

  #ifndef US100
  
  while (counter<samples) {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(5);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Read the signal from the sensor: a HIGH pulse whose
      // duration is the time (in microseconds) from the sending
      // of the ping to the reception of its echo off of an object.
      pinMode(echoPin, INPUT);
      noInterrupts();
      duration = pulseIn(echoPin, HIGH);
      interrupts();
      //With measured time we calculate the distance in cm:
      cm = (duration/2) * 0.034;
      collection[counter++] = cm;
      delay(50);  // mindest abstand zwischen zwei pings
  }

  // durchschnitt ermitteln, dabei addieren und mitzählen
   duration = 0;
  for (short i=0;i<samples;i++) {
    duration += collection[i];
  }
  average = duration/samples;

  duration = 0;
  counter = 0;
  for (short i=0;i<samples;i++) {
    if (collection[i] < average) {
      duration += collection[i];
      counter++;
    }  
  }  

  if (counter == 0) counter = 1;
  result = duration / counter;

  #endif
  return result;

}

void helloWorld(const char *HelloWorld)
{
  //Serial.println("helloWorld");
  display.setRotation(0);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t x = (display.width() - tbw) / 2;
  uint16_t y = (display.height() + tbh) / 2; // y is base line!
  display.setFullWindow();
  display.firstPage();
  display.fillScreen(GxEPD_WHITE);
  display.setCursor(x, y);
  display.print(HelloWorld);
  
  while (display.nextPage());
  //Serial.println("helloWorld done");

  display.powerOff();
}

void ShowError(char *HelloWorld)
{
  display.setRotation(90);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setPartialWindow(50, 0, display.width(), 20);
  display.firstPage();
  display.fillScreen(GxEPD_WHITE);
  display.setCursor(55, 10);

  display.print(HelloWorld);

  while (display.nextPage());
  //Serial.println("helloWorld done");

  display.powerOff();
}

static const uint16_t input_buffer_pixels = 640; // may affect performance

static const uint16_t max_row_width = 640; // for up to 7.5" display
static const uint16_t max_palette_pixels = 256; // for depth <= 8

uint8_t input_buffer[3 * input_buffer_pixels]; // up to depth 24
uint8_t output_row_mono_buffer[max_row_width / 8]; // buffer for at least one row of b/w bits
uint8_t output_row_color_buffer[max_row_width / 8]; // buffer for at least one row of color bits
uint8_t mono_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 b/w
uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w


void showPartialUpdate(bool Bewegung)
{
  uint16_t box_x = 5;
  uint16_t box_y = 5;
  uint16_t box_w = 15;
  uint16_t box_h = 15;
    uint16_t incr = display.epd2.hasFastPartialUpdate ? 1 : 3;
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);

  display.setPartialWindow(box_x, box_y, box_w, box_h);
    display.firstPage();
    do
    {
      if (Bewegung)
        display.fillRect(box_x, box_y, box_w, box_h, GxEPD_BLACK);
      else
        display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);  
    }
    while (display.nextPage());

}


void SendMessage(short Message) {
  char logString[64];
  WiFiClient client;
  bool connection_ok = false;

  
  sprintf(logString,"/4DAction/Strom?Job=Alert&Message=%d", Message);
  Serial.println(logString);

  if (!client.connect("192.168.0.34", httpPort))
  {
    Serial.println("connection failed");
    return;
  }
  client.print(String("GET ") + logString + " HTTP/1.1\r\n" +
               "Host: " + "192.168.0.34" + "\r\n" +
               "User-Agent: GxEPD2_Spiffs_Loader\r\n" +
               "Connection: close\r\n\r\n");
               

/*
  if (!client.connect("192.168.0.83", 80))
  {
    Serial.println("connection failed");
    return;
  }
  client.print(String("GET /1?t=240") + " HTTP/1.1\r\n" +
               "Host: " + "192.168.0.83" + "\r\n" +
               "User-Agent: GxEPD2_Spiffs_Loader\r\n" +
               "Connection: close\r\n\r\n");    
*/
}


// not called for Gang2
void ReportBewegung(short Bewegung, long distance) {
  char logString[64];
  WiFiClient client;
  bool connection_ok = false;

  
  sprintf(logString,"/4DAction/Strom?Job=GangBewegung&Bewegung=%d&distance=%d", Bewegung, distance);
  Serial.println(logString);

  if (!client.connect("192.168.0.34", httpPort))
  {
    Serial.println("connection failed");
    return;
  }
  client.print(String("GET ") + logString + " HTTP/1.1\r\n" +
               "Host: " + "192.168.0.34" + "\r\n" +
               "User-Agent: GxEPD2_Spiffs_Loader\r\n" +
               "Connection: close\r\n\r\n");
               

/*
  if (!client.connect("192.168.0.83", 80))
  {
    Serial.println("connection failed");
    return;
  }
  client.print(String("GET /1?t=240") + " HTTP/1.1\r\n" +
               "Host: " + "192.168.0.83" + "\r\n" +
               "User-Agent: GxEPD2_Spiffs_Loader\r\n" +
               "Connection: close\r\n\r\n");    
*/
}

void showBitmapBufferFrom_HTTP(const char* host, const char* path, const char* filename, int16_t x, int16_t y, bool with_color)
{
  display.setRotation(0);
  WiFiClient client;
  bool connection_ok = false;
  bool valid = false; // valid format to be handled
  bool flip = true; // bitmap is stored bottom-to-top
  uint32_t startTime = millis();
  Serial.println(); Serial.print("downloading file \""); Serial.print(filename);  Serial.println("\"");
  Serial.print("connecting to "); Serial.println(host);
  if (!client.connect(host, httpPort))
  {
    Serial.println("connection failed");
    return;
  }
  Serial.print("requesting URL: ");
  Serial.println(String("http://") + host + path + filename);
  client.print(String("GET ") + path + filename + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: GxEPD2_Spiffs_Loader\r\n" +
               "Connection: close\r\n\r\n");
  Serial.println("request sent");
  myDelay(3000);
  if ((x >= display.width()) || (y >= display.height())) return;
  while (client.connected())
  {
    String line = client.readStringUntil('\n');
    if (!connection_ok)
    {
      connection_ok = line.startsWith("HTTP/1.1 200 OK");
      if (connection_ok) Serial.println(line);
      //if (!connection_ok) Serial.println(line);
    }
    if (!connection_ok) Serial.println(line);
      //Serial.println(line);
    if (line == "\r")
    {
      Serial.println("headers received");
      break;
    }
  }

  // data received
  //Serial.println("data received");
  if (!connection_ok) return;
  // Parse BMP header
  uint16_t result = read16(client);
  if (result == 0x4D42) // BMP signature
  {
    uint32_t fileSize = read32(client);
    uint32_t creatorBytes = read32(client);
    uint32_t imageOffset = read32(client); // Start of image data
    uint32_t headerSize = read32(client);
    uint32_t width  = read32(client);
    uint32_t height = read32(client);
    uint16_t planes = read16(client);
    uint16_t depth = read16(client); // bits per pixel
    uint32_t format = read32(client);
    uint32_t bytes_read = 7 * 4 + 3 * 2; // read so far
    if ((planes == 1) && ((format == 0) || (format == 3))) // uncompressed is handled, 565 also
    {
      myDelay(1);
      Serial.print("File size: "); Serial.println(fileSize);
      Serial.print("Image Offset: "); Serial.println(imageOffset);
      Serial.print("Header size: "); Serial.println(headerSize);
      Serial.print("Bit Depth: "); Serial.println(depth);
      Serial.print("Image size: ");
      Serial.print(width);
      Serial.print('x');
      Serial.println(height);
      // BMP rows are padded (if needed) to 4-byte boundary
      uint32_t rowSize = (width * depth / 8 + 3) & ~3;
      if (depth < 8) rowSize = ((width * depth + 8 - depth) / 8 + 3) & ~3;

      if (height < 0)
      {
        height = -height;
        flip = false;
      }
      uint16_t w = width;
      uint16_t h = height;
      uint16_t outbuffer_idx = 0;
      if ((x + w - 1) >= display.width())  w = display.width()  - x;
      if ((y + h - 1) >= display.height()) h = display.height() - y;
      if (w <= max_row_width) // handle with direct drawing
      {
        valid = true;
        uint8_t bitmask = 0xFF;
        uint8_t bitshift = 8 - depth;
        uint16_t red, green, blue;
        bool whitish, colored;
        if (depth == 1) with_color = false;
        if (depth <= 8)
        {
          if (depth < 8) bitmask >>= depth;
          bytes_read += skip(client, 54 - bytes_read); //palette is always @ 54
          for (uint16_t pn = 0; pn < (1 << depth); pn++)
          {
            blue  = client.read();
            green = client.read();
            red   = client.read();
            client.read();
            bytes_read += 4;
            whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
            colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
            if (0 == pn % 8) mono_palette_buffer[pn / 8] = 0;
            mono_palette_buffer[pn / 8] |= whitish << pn % 8;
            if (0 == pn % 8) color_palette_buffer[pn / 8] = 0;
            color_palette_buffer[pn / 8] |= colored << pn % 8;
          }
        }

        //display.clearScreen();
        uint32_t rowPosition = flip ? imageOffset + (height - h) * rowSize : imageOffset;
        //Serial.print("skip "); Serial.println(rowPosition - bytes_read);
        bytes_read += skip(client, rowPosition - bytes_read);
        for (uint16_t row = 0; row < h; row++, rowPosition += rowSize) // for each line
        {
          if (!connection_ok || !client.connected()) break;
          myDelay(1); // yield() to avoid WDT
          uint32_t in_remain = rowSize;
          uint32_t in_idx = 0;
          uint32_t in_bytes = 0;
          uint8_t in_byte = 0; // for depth <= 8
          uint8_t in_bits = 0; // for depth <= 8
          uint8_t out_byte = 0xFF; // white (for w%8!=0 boarder)
          uint8_t out_color_byte = 0xFF; // white (for w%8!=0 boarder)
          uint32_t out_idx = 0;
          for (uint16_t col = 0; col < w; col++) // for each pixel
          {
            yield();
            if (!connection_ok || !client.connected()) break;
            // Time to read more pixel data?
            if (in_idx >= in_bytes) // ok, exact match for 24bit also (size IS multiple of 3)
            {
              uint32_t get = in_remain > sizeof(input_buffer) ? sizeof(input_buffer) : in_remain;
              uint32_t got = read(client, input_buffer, get);
              while ((got < get) && connection_ok)
              {
                //Serial.print("got "); Serial.print(got); Serial.print(" < "); Serial.print(get); Serial.print(" @ "); Serial.println(bytes_read);
                uint32_t gotmore = read(client, input_buffer + got, get - got);
                got += gotmore;
                connection_ok = gotmore > 0;
              }
              in_bytes = got;
              in_remain -= got;
              bytes_read += got;
            }
            if (!connection_ok)
            {
              Serial.print("Error: got no more after "); Serial.print(bytes_read); Serial.println(" bytes read!");
              break;
            }
            switch (depth)
            {
              case 24:
                blue = input_buffer[in_idx++];
                green = input_buffer[in_idx++];
                red = input_buffer[in_idx++];
                whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                break;
              case 16:
                {
                  uint8_t lsb = input_buffer[in_idx++];
                  uint8_t msb = input_buffer[in_idx++];
                  if (format == 0) // 555
                  {
                    blue  = (lsb & 0x1F) << 3;
                    green = ((msb & 0x03) << 6) | ((lsb & 0xE0) >> 2);
                    red   = (msb & 0x7C) << 1;
                  }
                  else // 565
                  {
                    blue  = (lsb & 0x1F) << 3;
                    green = ((msb & 0x07) << 5) | ((lsb & 0xE0) >> 3);
                    red   = (msb & 0xF8);
                  }
                  whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                  colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                }
                break;
              case 1:
              case 4:
              case 8:
                {
                  if (0 == in_bits)
                  {
                    in_byte = input_buffer[in_idx++];
                    in_bits = 8;
                  }
                  uint16_t pn = (in_byte >> bitshift) & bitmask;
                  whitish = mono_palette_buffer[pn / 8] & (0x1 << pn % 8);
                  colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
                  in_byte <<= depth;
                  in_bits -= depth;
                }
                break;
            }
            if (whitish)
            {
              // keep white
            }
            else if (colored && with_color)
            {
              out_color_byte &= ~(0x80 >> col % 8); // colored
            }
            else
            {
              out_byte &= ~(0x80 >> col % 8); // black
            }
            if ((7 == col % 8) || (col == w - 1)) // write that last byte! (for w%8!=0 boarder)
            {
              output_row_color_buffer[out_idx] = out_color_byte;
              output_row_mono_buffer[out_idx++] = out_byte;
              bmpbuffer[outbuffer_idx++] = out_byte;
              if (outbuffer_idx > 16000) 
                return;
              out_byte = 0xFF; // white (for w%8!=0 boarder)
              out_color_byte = 0xFF; // white (for w%8!=0 boarder)
            }
          } // end pixel
          int16_t yrow = y + (flip ? h - row - 1 : row);
          //display.writeImage(output_row_mono_buffer, output_row_color_buffer, x, yrow, w, 1);
        } // end line
        Serial.print("downloaded in ");
        Serial.print(millis() - startTime);
        Serial.println(" ms");

        //display.refresh();
;

        if (RedrawCounter++ > 15) {  // war 15;
             display.setFullWindow();
             RedrawCounter = 0;
        }
        else
          display.setPartialWindow(0, 0, display.width(), display.height());

      display.firstPage();
      do
      {
        display.fillScreen(GxEPD_BLACK);
        display.drawBitmap(0, 0, bmpbuffer, display.width(), display.height(), GxEPD_WHITE);
      }
      while (display.nextPage());

      
      }
      Serial.print("bytes read "); Serial.println(bytes_read);
    }
  }
  else
  {
    char message[100];
    sprintf(message,"Wrong signature %d",result);
    ReportError(message);
  }
  
  if (!valid)
  {
    ReportError("bitmap format not handled.");
  }
  display.powerOff();
}

void ReportError(char *errormessage) {
  Serial.println(errormessage);
  ShowError(errormessage);
}

uint16_t read16(WiFiClient& client)
{
  // BMP data is stored little-endian, same as Arduino.
  uint16_t result;
  ((uint8_t *)&result)[0] = client.read(); // LSB
  ((uint8_t *)&result)[1] = client.read(); // MSB
  return result;
}

uint32_t read32(WiFiClient& client)
{
  // BMP data is stored little-endian, same as Arduino.
  uint32_t result;
  ((uint8_t *)&result)[0] = client.read(); // LSB
  ((uint8_t *)&result)[1] = client.read();
  ((uint8_t *)&result)[2] = client.read();
  ((uint8_t *)&result)[3] = client.read(); // MSB
  return result;
}

uint32_t skip(WiFiClient& client, int32_t bytes)
{
  int32_t remain = bytes;
  uint32_t start = millis();
  while (client.connected() && (remain > 0))
  {
    if (client.available())
    {
      int16_t v = client.read();
      remain--;
    }
    else delay(1);
    if (millis() - start > 2000) break; // don't hang forever
  }
  return bytes - remain;
}

uint32_t read(WiFiClient& client, uint8_t* buffer, int32_t bytes)
{
  int32_t remain = bytes;
  uint32_t start = millis();
  while (client.connected() && (remain > 0))
  {
    if (client.available())
    {
      int16_t v = client.read();
      *buffer++ = uint8_t(v);
      remain--;
    }
    else delay(1);
    if (millis() - start > 5000) {
      Serial.println("Stop because timeout "); 
      break; // don't hang forever
    }
  }

  if (remain > 0) {
      Serial.print("remain handling: "); Serial.println(remain);
      while (client.available() && (remain > 0)) {
        int16_t v = client.read();
        *buffer++ = uint8_t(v);
        remain--;
      }
  }

  //Serial.print("requested: "); Serial.println(bytes);
  //Serial.print("remain: "); Serial.println(remain);

  return bytes - remain;
}