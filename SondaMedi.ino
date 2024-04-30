/**
 * I2 - SONDA "Enrico Medi"
 * @note Circuito Arduino incluso nella sonda1 per il rilevamento nella stratosfera di:
 *       - gas serra C02,O3,NO2,CH4
 *       - umidità,pressione,temperatura esterna ed interna
 *       - dati GPS
 *       - rotazione,accelerazione
 * @version 1.3
 * @date 2024-04-08
 * @author Andrea Vallorani <andrea.vallorani@polourbani.edu.it>
 * @copyright Infinity2 Project - Liceo "E. Medi" Montegiorgio (FM)
 */
//--- CONFIGURAZIONE ---
#define PIN_SD 10
#define PIN_DS18B20 7
#define PIN_CH4 A0
#define PIN_NO2 A3
#define RatioMQ4CleanAir (4.4) //RS / R0 = 60 ppm 


//--- LIBRERIE ---
//RTC
#include "RTC.h"
//SD
#include <SPI.h>
#include <SD.h>
//Matrice LED
//#include "Arduino_LED_Matrix.h"
//DS18B20 (Temperatura)
#include <OneWire.h>
#include <DS18B20_INT.h>
//SEN0536 (CO2)
#include <DFRobot_SCD4X.h>
//schermo OLED SSD1306 128x64 03C
#include <U8g2lib.h>
#include <Wire.h>
//BME280
#include <Adafruit_BME280.h>
//SEN0321 (O3)
#include "DFRobot_OzoneSensor.h"
//GPS NEO-6M-0-0U1
#include <NMEAGPS.h>
//#include <NeoSWSerial.h>

//MQ4
#include <MQUnifiedsensor.h>
MQUnifiedsensor MQ4("Arduino UNO",5,10,PIN_CH4,"MQ-4");

//-- VARIABILI GLOBALI
volatile bool irqFlag = false;
int config = 0;
String SEP = ",";
//ArduinoLEDMatrix matrix;
File logFile;  //file di log salvato nella microSD
RTCTime currentTime; //tempo corrente
OneWire oneWire(PIN_DS18B20);
DS18B20_INT sensoreTempD = DS18B20_INT(&oneWire);
DFRobot_SCD4X SCD4X(&Wire,SCD4X_I2C_ADDR);
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0,U8X8_PIN_NONE);
Adafruit_BME280 bme280;
DFRobot_OzoneSensor ozone;
NMEAGPS gps;
gps_fix dati;
//NeoSWSerial gpsPort(3,4);
//#define GPS_PORT_NAME "NeoSWSerial(3,4)"
//SoftwareSerial gpsPort(3,4);
//Serial1 gpsPort;

//reboot arduino
void (*resetFunc)(void) = 0;
void reboot() {
  //delay(1000);
  //resetFunc();
}

bool startGPS(){
  //Serial.print("-> Avvio GPS: ");
  Serial1.begin(9600);
  //Serial.println("OK");
}

void getGPS(){
  unsigned long start = millis();
  do{
    if(gps.available(Serial1)) dati = gps.read();
  }
  while (millis() - start < 500);
}

bool startO3(){
  //Serial.print("-> Avvio O3: ");
  if(ozone.begin(0x73)){
    //Serial.println("OK");
    ozone.setModes(MEASURE_MODE_PASSIVE);
  } 
  //else Serial.println("NO");
}

String getO3(){
  //Serial.println(ozone.readOzoneData(20));
  return (String)ozone.readOzoneData(20);
}

bool startBME280(){
  //Serial.print("-> Avvio BME280: ");
  bme280.begin(0x76);
  //if(bme280.begin(0x76)) Serial.println("OK");
  //else Serial.println("NO"); 
  return true;
}

String getBME280(){
  return (String)bme280.readTemperature()+SEP+(String)(bme280.readPressure()/100.0)+SEP+(String)bme280.readHumidity();
} 

void setOled(String l1="",String l2="",String l3="",String l4="",String l5="",String l6="",String l7="",String l8=""){
  oled.clearBuffer();
  oled.setCursor(0,0);
  if(l1!="") oled.print(l1+"\n");
  oled.setCursor(0,7);
  if(l2!="") oled.print(l2);
  oled.setCursor(0,14);
  if(l3!="") oled.print(l3);
  oled.setCursor(0,21);
  if(l4!="") oled.print(l4);
  oled.setCursor(0,28);
  if(l5!="") oled.print(l5);
  oled.setCursor(0,35);
  if(l6!="") oled.print(l6);
  oled.setCursor(0,42);
  if(l7!="") oled.print(l7);
  oled.setCursor(0,49);
  if(l8!="") oled.print(l8);
  oled.sendBuffer();
}

bool startOled(){
  oled.begin();
  oled.setFont(u8g2_font_5x7_tf); 
  oled.setFontRefHeightExtendedText(); 
  oled.setDrawColor(1); 
  oled.setFontPosTop(); 
  oled.setFontDirection(0);
  setOled("Avvio ...");
  return true;
}

bool startSCD41(){
  //Serial.print("-> Avvio SCD041: ");
  if(SCD4X.begin()){
    SCD4X.setTempComp(3.7);
    SCD4X.setSensorAltitude(70);  
    SCD4X.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);
    //Serial.println("OK");
  } 
  //else Serial.println("NO"); 
  return true;
}

String getSCD41(){
  if(SCD4X.getDataReadyStatus()){
    DFRobot_SCD4X::sSensorMeasurement_t data;
    SCD4X.readMeasurement(&data);
    return (String)data.CO2ppm+SEP+(String)data.temp+SEP+(String)data.humidity;
  }
  else return "-"+SEP+"-"+SEP+"-";
}

bool startDS18B20(){
  //Serial.print("-> Avvio DS18B20:");
  while(!sensoreTempD.begin()){
    //Serial.println(" sensore DS18B20 non trovato. Riprovo ...");
    delay(2000);
  }
  sensoreTempD.setResolution(12);
  //if(sensoreTempD.isConnected(PIN_DS18B20)) Serial.println(" OK");
  delay(2000);
  return true;
}

//max time: 50ms
float getDS18B20(){
  sensoreTempD.requestTemperatures();
  uint32_t start = millis();
  while(!sensoreTempD.isConversionComplete() && millis()<start+50) delay(1);
  return sensoreTempD.getTempCentiC()/100.0;
}

bool startRTC() {
  //Serial.println("-> Avvio RTC");
  RTC.begin();
  RTCTime startTime(1,Month::MARCH,2024,11,20,00,DayOfWeek::FRIDAY,SaveLight::SAVING_TIME_ACTIVE);
  RTC.setTime(startTime);
  RTC.setPeriodicCallback(ogni2Sec,Period::ONCE_EVERY_2_SEC);
  return true;
}

int getRTC(){
  RTC.getTime(currentTime);
  //return (String)currentTime.getHour()+":"+(String)currentTime.getMinutes()+":"+(String)currentTime.getSeconds();
  return currentTime.getUnixTime();
}

/*bool startMatrice(){
  Serial.println("-> Avvio matrice");
  uint8_t logoI2[8][12] = {
    {0,1,1,1,1,0,0,0,1,1,1,0},
    {0,1,1,1,1,0,0,1,1,1,1,1},
    {0,0,1,1,0,0,0,1,1,0,1,1},
    {0,0,1,1,0,0,0,0,0,0,1,1},
    {0,0,1,1,0,0,0,0,0,1,1,0},
    {0,0,1,1,0,0,0,0,1,1,0,0},
    {0,1,1,1,1,0,0,1,1,1,1,1},
    {0,1,1,1,1,0,0,1,1,1,1,1}};
  matrix.begin();
  matrix.renderBitmap(logoI2,8,12);
  delay(2000);
  return true;
}*/

//creazione file nella scheda SD
bool crea_file() {
  String filename;
  unsigned int i = 1;
  bool ok = false;
  do{
    filename = (String) "log_" + i + ".txt";
    //Serial.print("-> Creo file "+filename);
    if(SD.exists(filename)){
      //Serial.println(" NO (già presente)");
      i++;
    }
    else ok=true;
  } while(!ok);
  //Serial.println(" OK");
  logFile = SD.open(filename,FILE_WRITE);
  setOled("Aperto file:",filename);
  delay(2000);
  //Serial.print("-> Apro file "+filename);
  //if(logFile) Serial.println(" OK");
  //else Serial.println(" OK");
  return true;
}

bool startSD(){
  //Serial.print("-> Avvio scheda SD: ");
  Sd2Card card;
  pinMode(PIN_SD,OUTPUT);
  if(!card.init(SPI_HALF_SPEED,PIN_SD)){
    //Serial.println("scheda SD non inserita");
    //reboot();  //se la scheda non è leggibile forzo il riavvio di Arduino
    return false;
  }
  /*switch (card.type()){
    case SD_CARD_TYPE_SD1: Serial.print("SD1"); break;
    case SD_CARD_TYPE_SD2: Serial.print("SD2"); break;
    case SD_CARD_TYPE_SDHC: Serial.print("SDHC"); break;
    default: Serial.println("Unknown");
  }*/
  SdVolume volume;
  if(!volume.init(card)){
    //Serial.println(" partizione FAT16/FAT32 non trovata. Formattare la scheda.");
    return false;
  }
  uint32_t volumesize;
  /*Serial.print(" FAT");
  Serial.print(volume.fatType(),DEC);
  Serial.print(" ");*/
  volumesize = volume.blocksPerCluster();  // clusters are collections of blocks
  volumesize *= volume.clusterCount();  // we'll have a lot of clusters
  volumesize /= 2048;  // SD card blocks are always 512 bytes (2 blocks are 1KB)
  //Serial.print((float)volumesize / 1024.0);
  //Serial.println("GB");
  SD.begin(10);
  crea_file();
  delay(2000);
  return true;
}

void startMQ4(){
  MQ4.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ4.setA(1012.7); MQ4.setB(-2.786); // Configure the equation to to calculate CH4 concentration
  MQ4.init(); 
  MQ4.setR0(2.40);
  MQ4.serialDebug(false);
}

float getMQ4(){
  MQ4.update(); // Update data, the arduino will read the voltage from the analog pin
  return MQ4.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
}

void setup() {
  //Serial.begin(9600); //avvio seriale PC
  startOled();
  delay(1500);
  //startMatrice();
  startGPS();
  startSD();
  startDS18B20();
  startSCD41();
  startBME280();
  startO3();
  startMQ4();
  startRTC();
}

void loop(){
  if(irqFlag){
    irqFlag = false;
    uint32_t start = millis();
    String row;
    int tempo=getRTC();
    String temp1=(String)getDS18B20();
    String co2=getSCD41();
    float ch4 = getMQ4();
    int no2 = analogRead(PIN_NO2);
    String bme = getBME280();
    String o3 = getO3();
    getGPS();
    float lat=0,lon=0;
    NeoGPS::time_t dt = dati.dateTime;
    String time = "";
    int tot = dati.satellites;
    int hdop = 0;
    double alt=0,head=0,sog=0;
    if(tot>0){
      if(dati.valid.location){
        lat=dati.latitudeL();
        lon=dati.longitudeL();
      }
      dt = dati.dateTime;
      time = (String)dt.date+""+dt.month+""+dt.year+""+dt.hours+""+dt.minutes+""+dt.seconds;
      head = dati.heading();
      alt = dati.alt.whole;
      sog = dati.speed_kph();
      hdop = dati.hdop;
    }
    String gpsrow1 = ""+(String)tot+SEP+hdop+SEP+lat+SEP+lon;
    String gpsrow2 = time;
    String gpsrow3 = head+SEP+alt+SEP+sog+SEP+(String)(millis()-start);

    row = tempo+SEP+temp1+SEP+co2+SEP+(String)ch4+SEP+(String)no2+SEP+bme+SEP+o3+SEP+gpsrow1+SEP+gpsrow2+SEP+gpsrow3;
    if(alt<300){
      setOled(tempo+SEP+temp1,
              co2,
              (String)ch4+SEP+(String)no2,
              bme,
              o3,
              gpsrow1,
              gpsrow2,
              gpsrow3);
    }
    else oled.clearBuffer();
    //scrivo SD
    logFile.println(row);
    if(tempo%20==0) logFile.flush();
    //scrivo seriale
    //Serial.print(row+SEP);
    //Serial.println(millis()-start);
  }
}

void ogni2Sec() {
  irqFlag = true;
}
