/*******************************************************************************
  Modern Cuckcoo Clock
  Eng. Deulis Antonio Pelegrin Jaime February 2020

  This is a very limited version

  -----------MODIFIED / ADDED PARTS---------------
  TR/izmir/Nisan/2022
  Modified by Dr.TRonik YouTube: https://youtu.be/afD9WCIdP5M
  -12h-->24h
  -Cuckoo animation settled between 9h - 21h
  -7219 brightness adjusted to hours
  -Added opening emoji
  -Added second animation(:)per second

  MD_MAX72xx library (and other libraries too)
  must be installed and configured for the
  LED  matrix type being used.  Refer documentation
  included  in the MD_MAX72xx  library or see this link:
  https://majicdesigns.github.io/MD_MAX72XX/page_hardware.html
  ------------------------------------------------

  Hardware Connections:
  GPIO 1 - Tx
  GPIO 3 - Rx

  GPIO 5 / D1- Rx (SoftSerial to comunicate with the DFPlayer mini)
  GPIO 4 / D2- Tx (SoftSerial to comunicate with the DFPlayer mini)

  GPIO 0 /  D3 - LED Blue
  GPIO 2 /  D4 - LED Green
  GPIO 15 / D8- LED Red

  GPIO 12 / D6- CS  of Led Matrix Display
  GPIO 13 / D7- DIN of Led Matrix Display
  GPIO 14 / D5- CLK of Led Matrix Display

  GPIO 16 / D0 - Servo
*******************************************************************************/

/********************************************************************
  GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___
 ********************************************************************/
//ESP8266 ile ilgili kütüphaneler
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//MAX7219 led matrix ile ilgili kütüphaneler
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

//Mini player mp3 çalışması ile ilgili kütüphaneler
#include <SoftwareSerial.h>
#include <DFPlayerMini_Fast.h>

//SG90 Servo ile ilgili kütüphane
#include <Servo.h>

//İnternet üzerinden zamanı alabilme kütüphaneleri
#include <NTPClient.h>
#include <Time.h>
#include <TimeLib.h>
#include "Timezone.h"

#define STA_SSID "Dr.TRonik Youtube"
#define STA_PASSWORD  "Abone Olabilirsiniz"

#define SOFTAP_SSID "Cuckoo"
#define SOFTAP_PASSWORD "12345678"
#define SERVER_PORT 2000

#define NTP_OFFSET   60 * 60      // In seconds
#define NTP_INTERVAL 60 * 1000    // In miliseconds
#define NTP_ADDRESS  "tr.pool.ntp.org"  // change this to whatever pool is closest (see ntp.org)

#define LED_RED 15
#define LED_GREEN 2
#define LED_BLUE 0

#define SERVO_PIN 16
#define SERVO_MIN 5
#define SERVO_DANCE 160
#define SERVO_MAX 180

#define HARDWARE_TYPE MD_MAX72XX::ICSTATION_HW
#define MAX_DEVICES  4
#define CLK_PIN   14  // or SCK
#define DATA_PIN  13  // or MOSI
#define CS_PIN    12  // or SS

#define SOUND_SERIAL_RX 5
#define SOUND_SERIAL_TX 4

WiFiServer server(SERVER_PORT);

// SPI hardware interface
MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

SoftwareSerial sound_serial(SOUND_SERIAL_RX, SOUND_SERIAL_TX); // RX, TX
DFPlayerMini_Fast sound;

Servo servor;  // create servo object to control a servo

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

bool connected = false;
unsigned long last_second;

enum colors {
  color_black, color_blue, color_green, color_cyan, color_red, color_magenta, color_yellow, color_white
};

String st;
unsigned long cuckoo_animation_start = millis();
int cuckoo_animation_index = 0;
int cuckoo_animation_hours = 0;
int color_index = 1;
byte rgb = 0;
int h;
int  Brightness ;

/********************************************************************
  SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___
 ********************************************************************/
void setup() {
  P.begin();
  sound_serial.begin(9600);
  sound.begin(sound_serial);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  light(color_black);

  servor.attach(SERVO_PIN);
  servor.write(SERVO_MIN);

  last_second = millis();
  Serial.begin(115200);

  P.print("   ^_^"); //Açılış mesajı
  P.displayText("", PA_CENTER, 0, 0, PA_NO_EFFECT, PA_NO_EFFECT); //Max7219 gösterim ilk ayarlarını buradan alıyor...
  delay(1000); //MP3 çalarımız biraz geç hazırlanabiliyor, bekleme verelim...
  sound.volume(28); //Ses ayarı 0-30 arası...
  delay(50);
  sound.play(1); //Micro SD içindeki 001.mp3 parçasını çal! Kart içindeki parça ismi bu şekilde olmalı...

  bool r;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(IPAddress(192, 168, 73, 1), IPAddress(192, 168, 73, 1), IPAddress(255, 255, 255, 0));
  r = WiFi.softAP(SOFTAP_SSID, SOFTAP_PASSWORD, 6, 0);
  server.begin();

  timeClient.begin();   // Start the NTP UDP client

  if (r)
    Serial.println("SoftAP started!");
  else
    Serial.println("ERROR: Starting SoftAP");
  Serial.print("Trying WiFi connection to ");
  Serial.println(STA_SSID);

  WiFi.setAutoReconnect(true);
  WiFi.begin(STA_SSID, STA_PASSWORD);

  ArduinoOTA.begin();
}
bool go_to_sleep = false;
bool allow_animation = true;

/********************************************************************
  LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__
 ********************************************************************/
void loop() {
  P.displayAnimate();


  P.setIntensity(set_bright()); //Ekran parlaklığı, set_bright() fonksiyonundan dönen int değeri ile ayarlandı...
  delay(10); //Olmazsa olmaz!..

  //Handle OTA
  ArduinoOTA.handle();

  //Handle Connection to Access Point
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!connected)
    {
      connected = true;
      Serial.println("");
      Serial.print("Connected to ");
      Serial.println(STA_SSID);
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
  }
  else
  {
    if (connected)
    {
      connected = false;
      Serial.print("Disonnected from ");
      Serial.println(STA_SSID);
    }
  }

  if (millis() - last_second > 1000)
  {
    last_second = millis();



    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();

    // convert received time stamp to time_t object
    time_t local, utc;
    utc = epochTime;

    // Then convert the UTC UNIX timestamp to local time
    TimeChangeRule usEDT = { "EDT", Second, Sun, Mar, 2, +120};  //Eastern Daylight Time (EDT)... Türkiye: sabit +UTC+3 nedeni ile  +2saat = +120dk ayarlanmalı
    TimeChangeRule usEST = { "EST", First, Sun, Nov, 2,  };   //Eastern Time Zone: UTC - 6 hours - change this as needed//Türkiye için değil...
    Timezone usEastern(usEDT, usEST);
    local = usEastern.toLocal(utc);

    h = hour(local); //Eğer 24 değil de 12 li saat istenirse :hourFormat12(local); olmalı
    int m = minute(local);
    int h_cuckoo = hourFormat12(local); //Akşam 9 = 21 defa guguk ötmesin...

    st = "";
    st += h;

    //st += ":";
    //____
    if (millis() / 1000 % 2 == 0) // her 1 saniye için
    {
      st += ":"; //iki noktayı göster
    }
    else
    {
      st += " "; // gösterme
    }
    //___
    if (m < 10)  // 0 ---> 00
      st += "0";
    st += m;

    P.print(st);

    if ((m == 0) && (h >= 9 && h < 22)) //Saat başlarında ve eğer saat 9-21 arası ise animasyon...
    {
      if (allow_animation)
      {
        allow_animation = false;
        cuckoo_animation_hours = h_cuckoo;
        cuckoo_animation_index = 0;
        cuckoo_animation_start = millis();
      }
    }
    else
    {
      allow_animation = true;
    }

  }

  P.displayAnimate();

  unsigned long elapsed = millis() - cuckoo_animation_start;

  if (cuckoo_animation_hours > 0)
  {
    if (cuckoo_animation_index == 0)
    {
      color_index++;
      if (color_index > 7)
        color_index = 1;
      light(color_index);

      servor.write(SERVO_DANCE);
      sound.play(1);
      cuckoo_animation_index++;
    }
    else if (cuckoo_animation_index == 1 && elapsed > 300)
    {
      servor.write(SERVO_MAX);
      cuckoo_animation_index++;
    }
    else if (cuckoo_animation_index == 2 && elapsed > 500)
    {
      servor.write(SERVO_DANCE);
      cuckoo_animation_hours--;
      cuckoo_animation_index++;

      if (cuckoo_animation_hours == 0)
      {
        go_to_sleep = true;
        cuckoo_animation_start = millis();
      }
    }
    else if (cuckoo_animation_index == 3 && elapsed > 1500)
    {
      cuckoo_animation_index = 0;
      cuckoo_animation_start = millis();
    }
  }
  else if (go_to_sleep && elapsed > 500)
  {
    go_to_sleep = false;
    servor.write(SERVO_MIN);
    light(0);
  }
}

/********************************************************************
  VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs
********************************************************************/
void light(int color)
{
  if (color > 7 || color < 0)
    color = 0;

  bool b = (color & 0x01) == 0x01;
  bool g = (color & 0x02) == 0x02;
  bool r = (color & 0x04) == 0x04;

  digitalWrite(LED_RED, r);
  digitalWrite(LED_GREEN, g);
  digitalWrite(LED_BLUE, b);
}

void bird_in()
{
  servor.write(SERVO_MIN);
}

void bird_out()
{
  servor.write(SERVO_MAX);
}

int set_bright () {
  //Saate göre parlaklık  ayarlama
  if (h >= 8 && h < 22)
  {
    Brightness = 15;
  }

  else
  {
    Brightness = 1;
  }
  
  return Brightness;
}
