/*******************************************************************************
     ____       __________              _ __  
    / __ \_____/_  __/ __ \____  ____  (_) /__
   / / / / ___/ / / / /_/ / __ \/ __ \/ / //_/
  / /_/ / /  _ / / / _, _/ /_/ / / / / / ,<   
 /_____/_/  (_)_/ /_/ |_|\____/_/ /_/_/_/|_|  

 Kasım 2024 İzmir MAX7219 4'lü modul ile NTP saat, derece, nem, takvim, kayar yazı
 https://youtu.be/YZslXzsWp9M
  
  Hardware Connections:    
  GPIO 12 / D6- CS  of Led Matrix Display
  GPIO 13 / D7- DIN of Led Matrix Display
  GPIO 14 / D5- CLK of Led Matrix Display
  GPIO  4 / D2- DHT11
*******************************************************************************/

/********************************************************************
  GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___
 ********************************************************************/
//ESP8266 ile ilgili kütüphaneler
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "DHT.h"

//MAX7219 led matrix ile ilgili kütüphaneler
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

//İnternet üzerinden zamanı alabilme
#include <NTPClient.h>
#include <Time.h>
#include <TimeLib.h>
#include "Timezone.h"

//Tanım ve Nesne'ler
#define STA_SSID "YourSSID"
#define STA_PASSWORD "YourPass"

#define NTP_OFFSET 60 * 60             // In seconds
#define NTP_INTERVAL 60 * 1000         // In miliseconds
#define NTP_ADDRESS "tr.pool.ntp.org"  // change this to whatever pool is closest (see ntp.org)


#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
/*MAX7219 Led Matrix Donanım Tipleri:
  1 GENERIC_HW
  2 FC16_HW 
  3 PAROLA_HW
  4 ICSTATION_HW 
*/
#define MAX_DEVICES 4
#define CLK_PIN 14   // or SCK
#define DATA_PIN 13  // or MOSI
#define CS_PIN 12    // or SS

#define DHTPIN 4       //DHT pin tanımlaması GPIO 4 / D2
#define DHTTYPE DHT11  //DHT modeli tanımlaması
DHT dht(DHTPIN, DHTTYPE);

//WiFiServer server(SERVER_PORT);

// SPI hardware interface
MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

//Değişkenler
bool connected = false;
unsigned long timer, timer1, timer2;

String st;
//String total;

int h, m, g, a, y, w;
int Brightness;
int hum, temp;

char gun_isimleri[8][10] = { " ", "PAZAR", "PAZARTESi", "SALI", "CARSAMBA", "PERSEMBE", "CUMA", "CUMARTESi" };
char ay_isimleri[13][8] = { " ", "OCAK", "SUBAT", "MART", "NiSAN", "MAYIS", "HAZiRAN", "TEMMUZ", "AGUSTOS", "EYLuL", "EKiM", "KASIM", "ARALIK" };


/********************************************************************
  SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___
 ********************************************************************/
void setup() {
  P.begin();
  dht.begin();

  timer = millis();
  timer1 = millis();
  timer2 = millis();
  Serial.begin(9600);

  P.print("   ^_^");
  delay(1000);
  P.print("RonaeR");                                               //Açılış mesajı
  P.displayText("", PA_CENTER, 0, 0, PA_NO_EFFECT, PA_NO_EFFECT);  //Max7219 gösterim ilk ayarlar...

  WiFi.mode(WIFI_STA);

  timeClient.begin();  // Start the NTP UDP client

  Serial.print("Trying WiFi connection to ");
  Serial.println(STA_SSID);

  WiFi.setAutoReconnect(true);
  WiFi.begin(STA_SSID, STA_PASSWORD);

  ArduinoOTA.begin();

  hum = dht.readHumidity();      //Nem değeri
  temp = dht.readTemperature();  //Sıcaklık değeri
  delay(1000);
}

/********************************************************************
  LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__
 ********************************************************************/
void loop() {
  P.displayAnimate();
  P.setIntensity(set_bright());  //Ekran parlaklığı, set_bright() fonksiyonundan dönen int değeri ile ayarlandı...
  delay(10);                     //Olmazsa olmaz!..

  //Handle OTA
  ArduinoOTA.handle();

  //Handle Connection to Access Point
  if (WiFi.status() == WL_CONNECTED) {
    if (!connected) {
      connected = true;
      Serial.println("");
      Serial.print("Connected to ");
      Serial.println(STA_SSID);
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
  } else {
    if (connected) {
      connected = false;
      Serial.print("Disonnected from ");
      Serial.println(STA_SSID);
    }
  }

  if (millis() - timer1 > 60 * 1000) {
    timer1 = millis();

    hum = dht.readHumidity();      //Nem değeri
    temp = dht.readTemperature();  //Sıcaklık değeri
    Serial.println(temp);
  }

  if (millis() - timer > 1000) {
    timer = millis();

    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();

    // convert received time stamp to time_t object
    time_t local, utc;
    utc = epochTime;

    // Then convert the UTC UNIX timestamp to local time
    TimeChangeRule usEDT = { "EDT", Second, Sun, Mar, 2, +120 };  //Eastern Daylight Time (EDT)... Türkiye: sabit +UTC+3 nedeni ile  +2saat = +120dk ayarlanmalı
    TimeChangeRule usEST = { "EST", First, Sun, Nov, 2, +120 };
    /*Eastern Time Zone:    
   Ülkemizde yaz kış saati uygulaması olmadığından;
   Kasım (~ ilk pazarı) ayında değişen EST (Eastern Time Zone) için:
   TimeChangeRule usEST = { "EST", First, Sun, Nov, 2,  }; 
   satırını şu şekilde değiştirin: 
   TimeChangeRule usEST = { "EST", First, Sun, Nov, 2, +120 };  
   Saat bilgisinin iki saat geriden gelmesi ortadan kalkacaktır...

   Mart ayı içerisinde (~12.Mart) yeniden değişim olacağından,
   TimeChangeRule usEST = { "EST", First, Sun, Nov, 2, }; 
   Olacak şekilde bu değişiklik geri alınmalıdır...*/

    Timezone usEastern(usEDT, usEST);
    local = usEastern.toLocal(utc);

    h = hour(local);  //Eğer 24 değil de 12 li saat istenirse :hourFormat12(local); olmalı
    m = minute(local);
    g = day(local);
    a = month(local);
    y = year(local);
    w = weekday(local);

    st = "";
    st += h;

    //st += ":";
    //____
    if (millis() / 1000 % 2 == 0)  // her 1 saniye için
    {
      st += ":";  //iki noktayı göster

    } else {
      st += " ";  // gösterme
    }
    //___
    if (m < 10)  // 0 ---> 00
      st += "0";
    st += m;

  }  //Time millis sonu
  //   //P.displayShutdown(true);

  if (m == 1 || m == 11 || m == 21 || m == 31 || m == 41 || m == 51) {
    //10dk.da bir kayar yazı bilgi ekranı takvim, derece, nem, mesaj...
    char charSlider[75];
    sprintf(charSlider, "<<< %02d.%-7s.%04d <<< %-9s <<< %02d'C.  Nem: %%%02d <<< %s ", g, ay_isimleri[a], y, gun_isimleri[w], temp, hum, "*The Ronaer's*");

    if (P.displayAnimate()) {
      P.displayText(charSlider, PA_CENTER, 83, 0, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
    }
  } else if (millis() - timer2 > 30 * 1000) {
    //30 snniyede bir derece ve nem bilgisi 5'er sn. ara ile arka arkaya ekranda...
    char chartemp[10];
    sprintf(chartemp, "%02d 'C.", temp);  // Çıktı örneği: 24 'C
    char charhum[10];
    sprintf(charhum, "%%%02d", hum);  // Çıktı örneği: %55
    P.print(chartemp);
    delay(5000);
    P.print(charhum);
    delay(5000);
    timer2 = millis();
  } else {
    P.print(st);
  }
}

/********************************************************************
  VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs
********************************************************************/
int set_bright() {
  /*Ay ve Saate göre gösterge parlaklık  ayarlama:
  Eylül, ekim, kasım ,aralık, ocak, şubat için saat 10 / 18 arası parlak
  Diğer aylar için saat 08 / 21 arası parlak */
  if ((a >= 9 && a < 13) || (a >= 1 && a < 3)) {
    if (h >= 10 && h < 13) {
      Brightness = 2;
    } else if (h >= 13 && h < 18) {
      Brightness = 6;
    } else {
      Brightness = 0;
    }
  } else {
    if (h >= 8 && h < 21) {
      Brightness = 8;
    } else {
      Brightness = 0;
    }
  }
  return Brightness;
}

/*___:
bilgi@ronaer.com
WhatsApp: https://whatsapp.com/channel/0029VaxtFPiLSmbgUryuGs0E
https://www.instagram.com/dr.tronik2023/   
YouTube: Dr.TRonik: www.youtube.com/c/DrTRonik
PCBWay: https://www.pcbway.com/project/member/shareproject/?bmbno=A0E12018-0BBC-4C
*/
