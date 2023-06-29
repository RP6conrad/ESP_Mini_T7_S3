
 /*
 * Project based on the https://github.com/RP6conrad/ESP-GPS-Logger/blob/master/README.md
 * Removed all display stuff, changes for the esp32 s3
 * https://wiki.seeedstudio.com/xiao_esp32s3_pin_multiplexing/
 * Lipo dimesnsions : 502030/603035/402030/103450/602244/582728/357095/582535/602535 
 */
#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>
#include <ETH.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h> 
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include <driver/gpio.h>
#include <lwip/apps/sntp.h>
#include <esp32-hal.h>
#include <time.h>
#include <EEPROM.h>
#include "Arduino.h"
#include "FS.h"
#include "SPI.h"
#include "sys/time.h"
#include "Ublox.h"
#include "SD_card.h"
#include "GPS_data.h"
#include "ESP32FtpServerJH.h"
#include "OTA_server.h" 
#include "Definitions.h"
#include "ESP_functions.h"

SPIClass sdSPI(FSPI);//was VSPI

const char *filename = "/config.txt";
const char *filename_backup = "/config_backup.txt"; 

void setup() {
  EEPROM.begin(EEPROM_SIZE);
  config.ublox_type = EEPROM.read(0);
  //pinMode(BUILD_IN_LED, OUTPUT);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(BUILD_IN_LED,0);
  ledcWrite(0,128);
  Serial.begin(115200);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only on the S3
  }
  delay(5000);
  Serial.println("setup Serial over USB ! ");
 ledcWrite(0,255);
  static const int spiClk = 1000000; // 1 MHz
  sdSPI.begin(SDCARD_CLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_SS);//default 20 MHz gezet worden !
  print_wakeup_reason(); //Print the wakeup reason for ESP32, go back to sleep is timer is wake-up source !

  pinMode(POWER1_ubx, OUTPUT);//Power beitian //default drive strength 2, only 2.7V @ ublox gps
  pinMode(POWER2_ubx, OUTPUT);//Power beitian
  
 int error=rtc_gpio_set_drive_capability(POWER1_ubx_NUM,GPIO_DRIVE_CAP_3);//see https://www.esp32.com/viewtopic.php?t=5840
 Serial.println(error);
 error=rtc_gpio_set_drive_capability(POWER2_ubx_NUM,GPIO_DRIVE_CAP_3);//3.0V @ ublox gps current 50 mA
 Serial.println(error);
 error=gpio_set_drive_capability(POWER1_ubx_NUM,GPIO_DRIVE_CAP_3);
 Serial.println(error);
 error=gpio_set_drive_capability(POWER2_ubx_NUM,GPIO_DRIVE_CAP_3);
 Serial.println(error);
  Ublox_on();//beitian bn220 power supply over output 15, 16
  Serial2.setRxBufferSize(1024); // increasing buffer size ?
  if((config.ublox_type==M8_38400BD)|(config.ublox_type==M9_38400BD)|  (config.ublox_type==M10_38400BD)){
    Serial2.begin(38400, SERIAL_8N1, RXD2, TXD2); //connection to ublox over Serial2 
    Serial.println("Serial 2 on 38400 bd");
    }
  else{
   Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); //connection to ublox over Serial2
   Serial.println("Serial 2 on 9600 bd");
   }   
  Serial.println("Serial2 Txd is on pin: "+String(TXD2));
  Serial.println("Serial2 Rxd is on pin: "+String(RXD2));
  for(int i=0;i<425;i++){//Startup string van ublox to serial, ca 424 char !!
     while (Serial2.available()) {
              Serial.print(char(Serial2.read()));
              }
     delay(2);   //was delay (1)
     }
  
  sdSPI.begin(SDCARD_CLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_SS);//default 20 MHz gezet worden !
  struct timeval tv = { .tv_sec =  0, .tv_usec = 0 };
  settimeofday(&tv, NULL);
  if (!SD.begin(SDCARD_SS, sdSPI)) {
        sdOK = false;Serial.println("No SDCard found!");
        } 
        else {
        sdOK = true;Serial.println("SDCard found!");
        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        uint64_t totalBytes=SD.totalBytes() / (1024 * 1024);
        uint64_t usedBytes=SD.usedBytes() / (1024 * 1024);
        freeSpace=totalBytes-usedBytes;
        Serial.printf("SD Card Size: %lluMB\n", cardSize); 
        Serial.printf("SD Total bytes: %lluMB\n", totalBytes); 
        Serial.printf("SD Used bytes: %lluMB\n", usedBytes); 
        Serial.printf("SD free space: %lluMB\n", totalBytes-usedBytes); 
        Serial.println(F("Loading configuration..."));// Should load default config 
        loadConfiguration(filename, filename_backup, config); // load config file
        Serial.print(F("Print config file...")); 
        printFile(filename); 
        } 
  config.ublox_type = EEPROM.read(0);
  if(config.ublox_type==0xFF) {
    Auto_detect_ublox();//only test for ublox type and baudrate if unknown in configuration
    if(config.ublox_type!=UBLOX_TYPE_UNKNOWN){
      EEPROM.write(0, config.ublox_type);
      EEPROM.commit();
      }
    else{
       Serial.println("Can't detect type and or baudrate of ublox....");
      }  
    }     
  if((config.ublox_type==M8_9600BD)|(config.ublox_type==M8_38400BD)){
    Init_ublox(); //switch to ubx protocol
    }
  if((config.ublox_type==M9_9600BD)|(config.ublox_type==M9_38400BD)|(config.ublox_type==M10_9600BD)|(config.ublox_type==M10_38400BD)){
    Init_ubloxM10(); //switch to ubx protocol, same for M9/M10
    }
  Serial.print("SW Ublox=");
  Serial.println(ubxMessage.monVER.swVersion);
  Serial.print ("HW Ublox=");
  Serial.println (ubxMessage.monVER.hwVersion);
  Serial.print ("Extensions Ublox= ");
  for(int i=0;i<6;i++){
    Serial.print (ubxMessage.monVER.ext[i].extension);
    Serial.print (", ");
    }
  Serial.println();  
  Serial.println (ubxMessage.monGNSS.default_Gnss);
  Serial.println (ubxMessage.monGNSS.enabled_Gnss);
  if((config.ublox_type==M8_9600BD)|(config.ublox_type==M8_38400BD)){
      Set_rate_ublox(config.sample_rate);//after reading config file !! 
      } 
  if((config.ublox_type==M9_9600BD)|(config.ublox_type==M9_38400BD)|(config.ublox_type==M10_9600BD)|(config.ublox_type==M10_38400BD)){
      Set_rate_ubloxM10(config.sample_rate);//after reading config file !! 
      }  
  
  const char* ssid = config.ssid; //WiFi SSID
  const char* password = config.password; //WiFi Password
  const char *soft_ap_ssid = "ESP32AP"; //accespoint ssid
  const char *soft_ap_password = "password"; //accespoint password
    
  WiFi.onEvent(OnWiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.softAP(soft_ap_ssid, soft_ap_password);
  WiFi.begin(ssid, password);
  Serial.print("T5 MAC adress: ");
  WiFi.macAddress(mac);
 
  // Wait for connection during 10s in station mode
  bool ota_notrunning=true;
  while ((WiFi.status() != WL_CONNECTED)&(SoftAP_connection==false)) {
    server.handleClient(); // wait for client handle, and update BAT Status, this section should be moved to the loop... 
    Update_bat();         //client counter wait until download is finished to prevent stoping the server during download
    if(digitalRead(WAKE_UP_GPIO)==false){//switch over to AP-mode, search time 100 s
       if(ota_notrunning){
            WiFi.disconnect();
            WiFi.mode(WIFI_AP);
            WiFi.softAP(soft_ap_ssid, soft_ap_password); 
            wifi_search=100;
            IP_adress =  WiFi.softAPIP().toString();
            Serial.println(IP_adress);
            Serial.println("start OTA Server");
            OTA_setup(); //start webserver and ota - for use on AP Mode
            ota_notrunning=false;
          }else{
            wifi_search=100;
            Serial.println("Set AP Counter to 100");
          }
        }
    delay(1000);
    Serial.print(".");
    wifi_search--;
    if(wifi_search<=0){
      server.close();
      IP_adress = "0.0.0.0";
      break;
      }
    }
  if((WiFi.status()== WL_CONNECTED)|SoftAP_connection){
      Serial.println("");
      Serial.print("Connected to ");
      Serial.println(ssid);
      Serial.print("IP address: ");
      Serial.println(IP_adress);
      Wifi_on=true;
      ftpSrv.begin("esp32","esp32");    //username, password for ftp
      OTA_setup();  //start webserver for over the air update
      }
  else{
      Serial.println("No Wifi connection !");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      Wifi_on=false;      
  }
  delay(100);
   //Create RTOS task, so logging and e-paper update are separated (update e-paper is blocking, 800 ms !!)
  xTaskCreate(
                    taskOne,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    &t1);            /* Task handle. */
 
  xTaskCreate(
                    taskTwo,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    10000,            /* Stack size in bytes was 10000, but stack overflow on task 2 ?????? now 20000. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    &t2);            /* Task handle. */
          
}


void loop() { 
  static bool watchdog_started=false;
  if(!watchdog_started){
     Serial.println("Configuring WDT...");
     esp_task_wdt_deinit();
     watchdog_started=true;
    }
  int wdt_task0_duration=millis()-wdt_task0;
  int wdt_task1_duration=millis()-wdt_task1;
  int task_timeout=(WDT_TIMEOUT -1)*1000;//1 second less then reboot timeout 
  if((wdt_task0_duration<task_timeout)&(wdt_task1_duration<task_timeout)){
    esp_err_t result = esp_task_wdt_init(WDT_TIMEOUT, true);//only way I found to reset the watchdog timer....
    //Serial.println(result);
    delay(100);
    }
  if((wdt_task0_duration>task_timeout)&(downloading_file)) {
    wdt_task0=millis();
    Serial.println("Extend watchdog_timeout due long download"); 
    }     
  if((wdt_task0_duration>task_timeout)&(!downloading_file)) Serial.println("Watchdog task0 triggered");
  if(wdt_task1_duration>task_timeout) Serial.println("Watchdog task1 triggered");
   
  delay(100); 
}

void taskOne( void * parameter )
{
 while(true){ 
  static int print_time1=0;
  if((millis()-print_time1>1000)){Serial.println("Task 1 is running");print_time1=millis();}
   wdt_task0=millis();
   
   if(Long_push_WakeUp.Button_pushed()) Shut_down();
   if (Short_push_WakeUp.Button_pushed()){
      config.field=Short_push_WakeUp.button_count;
      }
   
   if((WiFi.status() != WL_CONNECTED)&(Wifi_on==true)&(SoftAP_connection==false)){
        Serial.println("No Wifi connection !");
        server.close();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        Wifi_on=false; 
        sntp_stop();//no messing around with time synchronization anymore, EPOCH time is set to 0 !!! 
        setTimeZone(0,0);//set TZ to UTC, as mktime returns localtime !!!!
        printLocalTime();
        NTP_time_set=false;
       }   
   if((WiFi.status() == WL_CONNECTED)|SoftAP_connection){
        ftpSrv.handleFTP();        //make sure in loop you call handleFTP()!! 
        server.handleClient();
        ftpStatus=ftpSrv.cmdStatus;//for e-paper
        // Init and get the time if WLAN, create "Archive" dir if not present
        if(WiFi.status() == WL_CONNECTED){
          const char* ntpServer = "pool.ntp.org";
          long  gmtOffset_sec = 3600*config.timezone;//configTime sets Timezone !! mktime use TZ...
          const int daylightOffset_sec = 0;
          if(NTP_time_set==0){
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
            setTimeZone(-gmtOffset_sec, daylightOffset_sec);
            printLocalTime();
            //setTimeZone(0,0);
           // printLocalTime();
            NTP_time_set=true;
            }
          if(!SD.exists("/Archive")){
              SD.mkdir("/Archive");
              }
          }
        #if defined (STATIC_DEBUG)
        msgType = processGPS(); //debug purposes
        static int testtime;
        if((millis()-testtime)>1000){
              testtime=millis();
              Set_GPS_Time(config.timezone);
              config.timezone++;
              }
        #endif
        }
   else msgType = processGPS(); //only decoding if no Wifi connection
          //while (Serial2.available()) {
          //Serial.print(char(Serial2.read()));}  
 if ( msgType == MT_NAV_DOP ) {      //Logging after NAV_DOP, ublox sends first NAV_PVT, and then NAV_DOP.  
        if((ubxMessage.navPvt.numSV>=MIN_numSV_FIRST_FIX)&((ubxMessage.navPvt.sAcc/1000.0f)<MAX_Sacc_FIRST_FIX)&(ubxMessage.navPvt.valid>=7)&(GPS_Signal_OK==false)){
              GPS_Signal_OK=true;
              first_fix_GPS=millis()/1000;
              }
        if(GPS_Signal_OK==true){
              GPS_delay++;
              }
        if ((Time_Set_OK==false)&(GPS_Signal_OK==true)&(GPS_delay>(TIME_DELAY_FIRST_FIX*config.sample_rate))){//vertraging Time_Set_OK is nu 10 s!!
          int avg_speed=(avg_speed+ubxMessage.navPvt.gSpeed*19)/20; //FIR filter gem. snelheid laatste 20 metingen in mm/s
          if(avg_speed>MIN_SPEED_START_LOGGING){
            if(Set_GPS_Time(config.timezone)){            
                Time_Set_OK=true;
                Shut_down_Save_session=true;
                start_logging_millis=millis();
                Open_files();//only start logging if GPS signal is OK
                }
          }      //    Alleen speed>0 indien snelheid groter is dan 1m/s + sACC<1 + sat<5 + speed>35 m/s !!!
        }
        if ((sdOK==true)&(Time_Set_OK==true)&(nav_pvt_message>10)&(nav_pvt_message!=old_message)){
                  old_message=nav_pvt_message;
                  float sAcc=ubxMessage.navPvt.sAcc/1000;
                  gps_speed=ubxMessage.navPvt.gSpeed; //hier alles naar mm/s !!
                  static int last_flush_time=0;
                  last_gps_msg=millis();
                  if((millis()-last_flush_time)>60000){
                      Flush_files();
                      last_flush_time=millis();
                      }
                  if((ubxMessage.navPvt.numSV<=MIN_numSV_GPS_SPEED_OK)|((ubxMessage.navPvt.sAcc/1000.0f)>MAX_Sacc_GPS_SPEED_OK)|(ubxMessage.navPvt.gSpeed/1000.0f>MAX_GPS_SPEED_OK)){
                      gps_speed=0;
                      } 
                  Log_to_SD();//hier wordt ook geprint naar serial !!
                  Ublox.push_data(ubxMessage.navPvt.lat/10000000.0f,ubxMessage.navPvt.lon/10000000.0f,gps_speed);   
                  run_count=New_run_detection(ubxMessage.navPvt.heading/100000.0f,S2.avg_s); 
                  alfa_window=Alfa_indicator(M250,M100,ubxMessage.navPvt.heading/100000.0f);
                  if(run_count!=old_run_count)Ublox.run_distance=0;
                  old_run_count=run_count;        
                  M100.Update_distance(run_count);
                  M250.Update_distance(run_count);
                  M500.Update_distance(run_count);
                  M1852.Update_distance(run_count);
                  S2.Update_speed(run_count); 
                      
                  S10.Update_speed(run_count);
                  
                  S1800.Update_speed(run_count);
                  S3600.Update_speed(run_count);
                  A250.Update_Alfa(M250);
                  A500.Update_Alfa(M500);
                  
                  }     
      } 
     else if( msgType == MT_NAV_PVT )  { //order messages in ubx should be ascending !!!
          if(Time_Set_OK==true){
                nav_pvt_message++;
                }
          }      
      else if( msgType == MT_NAV_SAT )  { 
          nav_sat_message++;
          Ublox_Sat.push_SAT_info(ubxMessage.navSat);
          #if defined(STATIC_DEBUG_SAT)
              Serial.print ("Mean cno= ");Serial.println (Ublox_Sat.sat_info.Mean_mean_cno);
              Serial.print ("Max cno= ");Serial.println (Ublox_Sat.sat_info.Mean_max_cno);
              Serial.print ("Min cno= ");Serial.println (Ublox_Sat.sat_info.Mean_min_cno);
              Serial.print ("Sats= ");Serial.println (Ublox_Sat.sat_info.Mean_numSV);
              Serial.print("numSV ");Serial.println(ubxMessage.navSat.numSvs);
              
              for(int i=0;i<ubxMessage.navSat.numSvs;i++){
                Serial.print("gnssID "); Serial.print(ubxMessage.navSat.sat[i].gnssId);
                Serial.print("svID "); Serial.print(ubxMessage.navSat.sat[i].svId);
                Serial.print(" cno "); Serial.print(ubxMessage.navSat.sat[i].cno);
                Serial.print(" X4 "); Serial.println(ubxMessage.navSat.sat[i].X4,BIN);
              }
              
          #endif
      }       
  }   
}
 
void taskTwo( void * parameter)
{
  while(true){ 
    static int print_time=0;
    if((millis()-print_time>1000)){Serial.println("Task 2 is running");print_time=millis();}
    wdt_task1=millis();

   for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    ledcWrite(0, dutyCycle);
    delay(7);
    }
 
  for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
    ledcWrite(0, dutyCycle);
    delay(7);
    }
    Update_bat();
    if(RTC_voltage_bat<MINIMUM_VOLTAGE) low_bat_count++;
    else low_bat_count=0;
    if(long_push==true){
       Shut_down();
    }
    else if(low_bat_count>10){
       Shut_down();
    }
   
  }
}
