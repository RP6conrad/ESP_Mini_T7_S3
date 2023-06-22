#include "E_paper.h"
#include "Rtos5.h"

void Boot_screen(void){
  display.init(); 
  display.setRotation(1);
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeSansBold9pt7b);
  display.setCursor(offset,14);
  display.drawExampleBitmap(ESP_GPS_logo, offset+178, 0, 48, 48, GxEPD_BLACK);
  if(RTC_voltage_bat<MINIMUM_VOLTAGE){
    display.println("Go back to sleep...");
    display.setCursor(offset,28);
    display.println(SW_version);
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(offset,56);
    display.print("Voltage to low: ");
    display.println(RTC_voltage_bat);
    display.setCursor(offset,76);
    display.print("Please charge battery!");
    Bat_level_Simon(offset);
    display.update();
    }
  else {
    display.fillScreen(GxEPD_WHITE); 
    display.println("ESP-GPS Booting");
    display.setCursor(offset,28);
    display.println(SW_version);
    display.setCursor(offset,42);
    display.println("Need for speed!");
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(offset,70);
    display.println("One moment please");
    display.setCursor(offset,98);
    if(config.ublox_type==0xFF) display.println("Auto detect GPS type");
    if(config.ublox_type==UBLOX_TYPE_UNKNOWN)display.println("ublox not found ??  ");
    if(config.ublox_type==M8_9600BD)display.println("M8 9600bd");
    if(config.ublox_type==M8_38400BD)display.println("M8 38400bd");
    if(config.ublox_type==M10_9600BD)display.println("M10 9600bd");
    if(config.ublox_type==M10_38400BD)display.println("M10 38400bd");
    if(config.ublox_type==M9_9600BD)display.println("M9 9600bd");
    if(config.ublox_type==M9_38400BD)display.println("M9 38400bd");
    if(freeSpace>0){
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(offset,120);
      display.print("SD Free space: ");
      display.print(freeSpace);
      display.print("Mb");
      display.updateWindow(0,0,250,122,true); 
      }
    else{
      Bat_level_Simon(offset);
      display.update();
      }  
  }  
}
void Off_screen(int choice){//choice 0 = old screen, otherwise Simon screens
  //int offset=0;
  float session_time=(millis()-start_logging_millis)/1000 ;
  if(choice==0){
      display.setRotation(1);
      display.fillScreen(GxEPD_WHITE);
      display.setTextColor(GxEPD_BLACK);
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(offset,14);
      display.drawExampleBitmap(ESP_GPS_logo, offset+178, 0, 48, 48, GxEPD_BLACK);
      display.println("ESP-GPS Saving");
      display.setCursor(offset,28);
      display.println(SW_version);
      display.setCursor(offset,42);
      display.println("Need for speed!");
      display.setFont(&FreeSansBold12pt7b);
      display.setCursor(offset,70);
      if(Shut_down_Save_session==true){
        display.println("Saving your session..");
        }
      else{
        display.println("Going back to sleep..");
      }
      display.updateWindow(0,0,250,122,true);
      }
  else{
      Serial.println("Off_screen Simon");
      display.setRotation(1);
      display.fillScreen(GxEPD_WHITE);
      display.setTextColor(GxEPD_BLACK);
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(offset,14);
      display.drawExampleBitmap(ESP_GPS_logo, offset+178, 0, 48, 48, GxEPD_BLACK);
      display.println("ESP-GPS Saving");
      display.setCursor(offset,28);
      display.println(SW_version);
      display.setCursor(offset,42);
      display.println("Need for speed!");
      display.setFont(&FreeSansBold12pt7b);
      display.setCursor(offset,70);
      if(Shut_down_Save_session==true){
        display.println("Saving your session..");
        }
      else{
        display.println("Going back to sleep..");
      }
      display.updateWindow(0,0,250,122,true);
  }
  delay(10000);//om te voorkomen dat update opnieuw start !!!
}
//Screen in deepsleep, update bat voltage, refresh every 4000s !!
void Sleep_screen(int choice){
  if (offset>9)offset--;
  if (offset<1)offset++;
  display.init(); 
  display.setRotation(1);
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  if(choice==0){
      display.setFont(&FreeSansBold18pt7b);
      display.setCursor(offset,24);
      display.print("Dist: ");
      display.println(RTC_distance,0);
      display.setCursor(offset,56);
      display.print("AVG: ");display.println(RTC_avg_10s,2);
      display.setCursor(offset,88);
      display.print("2s: ");display.print(RTC_max_2s);//display.println("Hz"); 
      display.setCursor(offset,120);
      display.print("Bat: ");display.print(RTC_voltage_bat,2);
      Bat_level(offset+222,87);
      display.update();
      }
   else{
      int row1=15;
      int row=15;
      int row2=row1+row;
      int row3=row2+row;
      int row4=row3+row;
      int row5=row4+row;
      int row6=row5+row;
      int col1=0+offset;
      int col2=34+offset;
      int col3=90+offset;
      int col4=146+offset;
      int line=0;
      display.fillScreen(GxEPD_WHITE);
//      display.fillRect(0,0,255,122,GxEPD_WHITE);
    
    // Board Logo's:
    // add special logos - funlogos
    if(RTC_Board_Logo==11) // Schwechater - Austrian Beer - by tritondm
      display.drawExampleBitmap(epd_bitmap_Schwechater, 195, -10, 79, 132, GxEPD_BLACK);
    //Board logos
    if(RTC_Board_Logo==1)//Logo's Simon Dijkstra
      display.drawExampleBitmap(Starboard_logoS_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==2)  
      display.drawExampleBitmap(Fanatic_logoS_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==3)  
      display.drawExampleBitmap(JP_logoS_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==4)  
      display.drawExampleBitmap(NoveNove_logoS_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==5)//Logo's Jan Scholten      
      display.drawExampleBitmap(Mistral_logoS_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==6)
      display.drawExampleBitmap(Goya_logoS_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==7)
      display.drawExampleBitmap(Patrik_logoS_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==8)
      display.drawExampleBitmap(Severne_logoS_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==9)
      display.drawExampleBitmap(Tabou_logoS_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==10)
      display.drawExampleBitmap(F2_logo_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==12)
      display.drawExampleBitmap(FutureFly_logo_zwart, 195, 0, 48, 48, GxEPD_BLACK);
    if(RTC_Board_Logo==13)
      display.drawExampleBitmap(ESP_GPS_logo, 195, 0, 48, 48, GxEPD_BLACK);
    // Zeil Logo's:
    if(RTC_Sail_Logo==1)//Logo's Simon Dijkstra
      display.drawExampleBitmap(GAsails_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK);
    if(RTC_Sail_Logo==2)
      display.drawExampleBitmap(DuoTone_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK);
    if(RTC_Sail_Logo==3)
      display.drawExampleBitmap(NP_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK);
    if(RTC_Sail_Logo==4)
      display.drawExampleBitmap(Pryde_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK);
    if(RTC_Sail_Logo==5)//Logo's Jan Scholten
      display.drawExampleBitmap(Loftsails_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK);
    if(RTC_Sail_Logo==6)
      display.drawExampleBitmap(Gunsails_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK);
    if(RTC_Sail_Logo==7)
      display.drawExampleBitmap(Point7_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK);
    if(RTC_Sail_Logo==8)
      display.drawExampleBitmap(Simmer_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK);
    if(RTC_Sail_Logo==9)
      display.drawExampleBitmap(Naish_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK); 
    if(RTC_Sail_Logo==10) //Severne as Sail logo !!! 
      display.drawExampleBitmap(Severne_logoS_zwart, 195, 50, 48, 48, GxEPD_BLACK);

      display.setFont(&SF_Distant_Galaxy7pt7b);//font ??
      display.setCursor(col1,105);

      time_print(RTC_hour);display.print(":");time_print(RTC_min);display.print(" ");display.print(RTC_day);display.print("-");display.print(RTC_month);display.print("-");display.print(RTC_year);
      display.print("  "); display.print(SW_version);
    
      display.setCursor(col1,121);
      display.setFont(&SF_Distant_Galaxy9pt7b);
      display.print(RTC_Sleep_txt);
    
      display.setRotation(0);
      display.setCursor(30,249);//was 30, 249
      display.setFont(&FreeSansBold6pt7b);
      if((int)(calibration_speed*100000)==194) display.print("speed in knots");//1.94384449 m/s to knots !!!
      if((int)(calibration_speed*1000000)==3600) display.print("speed in km/h");
    
      display.setRotation(1);
    // left column
      display.setFont(&FreeMonoBold9pt7b);
      display.setCursor(col1,row1);
      display.print("AV:");
      display.setCursor(col1,row2);
      display.print("R1:");
      display.setCursor(col1,row3);
      display.print("R2:");
      display.setCursor(col1,row4);
      display.print("R3:");
      display.setCursor(col1,row5);
      display.print("R4:");
      display.setCursor(col1,row6);
      display.print("R5:");
    
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(col2,row1);
      display.println(RTC_avg_10s,2);
      display.setCursor(col2,row2);
      display.println(RTC_R1_10s,2);
      display.setCursor(col2,row3);
      display.println(RTC_R2_10s,2);
      display.setCursor(col2,row4);
      display.println(RTC_R3_10s,2);
      display.setCursor(col2,row5);
      display.println(RTC_R4_10s,2);
      display.setCursor(col2,row6);
      display.println(RTC_R5_10s,2);
    
    // right column
      display.setFont(&FreeMonoBold9pt7b);
      display.setCursor(col3,row1);
      display.print("2sec:");
      display.setCursor(col3,row2);
      display.print("Dist:");
      display.setCursor(col3,row3);
      display.print("Alph :");
      display.setCursor(col3,row4);
      display.print("1h:");//
      display.setCursor(col3,row5);
      display.print("NM:");
      display.setCursor(col3,row6);
      display.print("Bat :");
  
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(col4,row1);
      display.println(RTC_max_2s,2);
      display.setCursor(col4,row2);
      display.println(RTC_distance,2);
      display.setCursor(col4,row3);
      display.println(RTC_alp,2);
      display.setCursor(col4,row4);
      display.println(RTC_1h,2);//
      display.setCursor(col4,row5);
      display.println(RTC_mile,2); 
      display.setCursor(col4,row6);
      float bat_perc=100*(1-(4.2-RTC_voltage_bat)/(4.2-3.4));
      if (bat_perc<0) bat_perc=0;
      if (bat_perc>100) bat_perc=100;
      display.print(bat_perc,0); 
      display.println("%"); 
      display.update(); 
      }
}
//for print hour&minutes with 2 digits
void time_print(int time){
        if (time<10)display.print("0");
        display.print(time);
      }
void Bat_level(int X_offset,int Y_offset){
    float bat_symbol=0;
    display.fillRect(X_offset+3,Y_offset,6,3,GxEPD_BLACK);
    display.fillRect(X_offset,Y_offset+3,12,30,GxEPD_BLACK);//monitor=(4.2-RTC_voltage_bat)*26
    if(RTC_voltage_bat<4.2) {
        bat_symbol=(4.2-RTC_voltage_bat)*28;
        display.fillRect(X_offset+2,Y_offset+7,8,(int)bat_symbol,GxEPD_WHITE);
        }
    }
void Bat_level_Simon(int offset){
    float bat_perc=100*(1-(4.2-RTC_voltage_bat)/(4.2-3.4));
    if (bat_perc<0) bat_perc=0;
    if (bat_perc>100) bat_perc=100;

    int batW=8;
    int batL=15;
    int posX=250-batW-10;
    int posY=122-batL;
    int line=2;
    int seg=3;
    int segW=batW-2*line;
    int segL=(batL-0.25*batW-2*line-(seg-1))/seg;
    display.fillRect(offset+posX,posY,0.5*batW,0.25*batW,GxEPD_BLACK);//battery top
    display.fillRect(offset+posX-0.25*batW,posY+0.25*batW,batW,batL,GxEPD_BLACK);//battery body
    if (bat_perc<67) display.fillRect(offset+posX-0.25*batW+line, posY+0.25*batW+line,            segW, segL,GxEPD_WHITE);
    if (bat_perc<33) display.fillRect(offset+posX-0.25*batW+line, posY+0.25*batW+line+1*(segL+1), segW, segL,GxEPD_WHITE);
    if (bat_perc<1)  display.fillRect(offset+posX-0.25*batW+line, posY+0.25*batW+line+2*(segL+1), segW, segL,GxEPD_WHITE);
    if (bat_perc<100) display.setCursor(offset+193,120);
    else display.setCursor(offset+185,120);
    display.setFont(&FreeSansBold9pt7b);
    display.print(int(bat_perc)); display.print("%");
    }
void Sats_level(int offset){
    int circelL=5;
    int circelS=2;
    int posX=178+offset;
    int posY=122-(circelL+2*circelS);

    display.fillCircle ( posX, posY, circelL, GxEPD_BLACK);
    display.fillTriangle (posX, posY+(circelL+2*circelS), posX-circelL , posY+circelS, posX+circelL , posY+circelS,GxEPD_BLACK);
    display.fillCircle ( posX, posY, circelS, GxEPD_WHITE);

    display.setCursor(posX-25,posY+8);
    display.setFont(&FreeSansBold9pt7b);
    display.print(int(ubxMessage.navPvt.numSV));
    }     
void M8_M10(int offset){
    display.setCursor(offset+110,120);
    display.setFont(&FreeSansBold9pt7b);
    if(config.ublox_type==M8_9600BD)display.print("M8");
    if(config.ublox_type==M8_38400BD)display.print("M8");
    if(config.ublox_type==M9_9600BD)display.print("M9");
    if(config.ublox_type==M9_38400BD)display.print("M9");
    if(config.ublox_type==M10_9600BD)display.print("M10");
    if(config.ublox_type==M10_38400BD)display.print("M10");
    }     
void Time(int offset){
    display.setCursor(offset,120);
    display.setFont(&FreeSansBold9pt7b);
    time_print(RTC_hour);display.print(":");time_print(RTC_min);
    }     
void Speed_in_Unit(int offset){
    display.setRotation(0);
    display.setCursor(30,offset+239);//was 30, 249
    display.setFont(&FreeSansBold6pt7b);
    if((int)(calibration_speed*100000)==194) display.print("speed in knots");//1.94384449 m/s to knots !!!
    if((int)(calibration_speed*1000000)==3600) display.print("speed in km/h");
    display.setRotation(1);
    }     
void Update_screen(int screen){
    static int count,old_screen,update_delay;
    char time_now[16];
    char time_now_sec[16];
    getLocalTime(&tmstruct, 0);
    sprintf(time_now,"%02d:%02d",tmstruct.tm_hour,tmstruct.tm_min);
    sprintf(time_now_sec,"%02d:%02d:%02d",tmstruct.tm_hour,tmstruct.tm_min,tmstruct.tm_sec);
    update_epaper=1; //was zonder else
    if(count%20<10) offset++;
    else offset--; 
    if(offset>10)offset=0;
    if(offset<0)offset=0;
    display.fillScreen(GxEPD_WHITE); 

    if(screen==BOOT_SCREEN){
      update_delay=1000;
      display.drawExampleBitmap(ESP_GPS_logo, offset+178, 0, 48, 48, GxEPD_BLACK);
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(offset,14);
      display.println("ESP-GPS Config");
      display.setCursor(offset,28);
      display.println(SW_version);
      display.setFont(&FreeSansBold12pt7b);
      display.setCursor(offset,56);
      if(sdOK==true)
      display.println("SD OK!");
      else display.println("No SD!");
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(offset,76);
      if(config.ublox_type==0)display.println("ublox??");
      if(config.ublox_type==1)display.println("M8 9600bd");
      if(config.ublox_type==2)display.println("M8 38400bd");
      if(config.ublox_type==3)display.println("M10 9600bd");
      if(config.ublox_type==4)display.println("M10 38400bd");
      if(config.ublox_type==5)display.println("M9 9600bd");
      if(config.ublox_type==6)display.println("M9 38400bd");
      display.setCursor(offset,94);
      display.print(config.UBXfile);
      display.setFont(&FreeSansBold12pt7b);
      display.setCursor(offset,114);
      if(config.config_fail==0) display.print("Config OK");
      else  display.print("Config FAIL !");
      if(screen!=old_screen)count=0;//eerste keer full update
      Bat_level_Simon(offset);
      Sats_level(offset);
      Speed_in_Unit(offset);
      delay(1000);
    }
    if(screen==WIFI_ON){  
      update_delay=1000;
      if(count%20<10) offset++;
      else offset--;    
      display.drawExampleBitmap(ESP_GPS_logo, offset+178, 0, 48, 48, GxEPD_BLACK);
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(offset,14);
      display.println("ESP-GPS Connect");
      display.setCursor(offset,28);
      display.println(SW_version);
      display.setCursor(offset,42);
      if(sdOK==true)
      display.println("SDcard OK");
      else display.println("No SDcard!!!");
      if(Wifi_on==1){
        display.setCursor(offset,60);
        display.fillRect(0, 43, 180, 79,GxEPD_WHITE);//clear lower part
        display.setFont(&FreeSansBold12pt7b);
        display.print("Connect to :");
        display.setCursor(offset,82);
        if(SoftAP_connection==true) display.print("Wifi AP:ESP32AP");//ap mode
        else {
          display.print("Wifi: ");//station mode
          display.println(config.ssid);
           }
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(offset,104);
        display.print("http://");
        display.println(IP_adress);
        display.setCursor(offset,120);
        display.println("Use your browser");
        Bat_level_Simon(offset);
      }
      else {
        display.setFont(&FreeSansBold9pt7b);
        display.fillRect(0, 0, 180, 15,GxEPD_WHITE);
        display.setCursor(offset,14);
        display.print("ESP-GPS ready  Wifi off");
        display.setCursor(offset,60);
        display.print("SD Free space: ");
        display.print(freeSpace);
        display.print("Mb");        
        display.setFont(&FreeSansBold12pt7b);
        display.setCursor(offset,84);
        if(ubxMessage.navPvt.numSV<5){
          display.println("Waiting for Sats >=5");
          display.setCursor(offset,100);
          display.setFont(&FreeSansBold9pt7b);
          display.println("Please go outside");
          Bat_level_Simon(offset);
          Sats_level(offset);
        }
        else{
          display.println("Ready for action");
          display.setFont(&FreeSansBold9pt7b);
          display.setCursor(offset,100);
          display.print("Move faster than ");
          if((int)(calibration_speed*100000)==194) {
            display.print(config.stat_speed*1.94384449);
            display.print("kn");
          }
          if((int)(calibration_speed*1000000)==3600) {
            display.print(config.stat_speed*3.6);
            display.print("km/h");
          }
          Bat_level_Simon(offset);
          Sats_level(offset);
          M8_M10(offset);
          Speed_in_Unit(offset);
        }
      }
    }
    if(screen==WIFI_STATION){  
      update_delay=100;   
      display.drawExampleBitmap(ESP_GPS_logo, offset+178, 0, 48, 48, GxEPD_BLACK);
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(offset,14);
      display.println("ESP-GPS Connect");
      display.setCursor(offset,28);
      display.println(SW_version);
      display.setCursor(offset,42);
      if(sdOK==true) display.println("SDcard OK");
      else display.println("No SDcard!!!");
      display.fillRect(0, 43, 180, 79,GxEPD_WHITE);//clear lower part
      display.setFont(&FreeSansBold12pt7b);
      display.setCursor(offset,70);
      display.print("Create wifi AP");
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(offset,84);
      display.print("Use magnet<10s!  ");
      display.println(wifi_search); 
      Bat_level_Simon(offset);
      if(screen!=old_screen)count=0;//eerste keer full update 
    }
    if(screen==WIFI_SOFT_AP){  
      update_delay=500;   
      display.fillScreen(GxEPD_WHITE);
      display.drawExampleBitmap(ESP_GPS_logo, offset+178, 0, 48, 48, GxEPD_BLACK);
      display.setFont(&FreeSansBold9pt7b);
      display.setCursor(offset,14);
      display.println("Connect to ESP-GPS");
      display.setCursor(offset,28);
      display.println(SW_version);
      display.setCursor(offset,42);
      if(sdOK==true)
      display.println("SDcard OK");
      else display.println("No SDcard!!!");
      display.setFont(&FreeSansBold12pt7b);
      display.setCursor(offset,70);
      display.print("Wifi AP:  ");
      display.println("ESP32AP ");
      display.setCursor(offset,88);
      display.print("password ");
      display.setCursor(offset,120);
      display.print(IP_adress);
      display.print("  ");
      display.print(wifi_search);   
      Bat_level_Simon(offset);
      if(screen!=old_screen)count=0;//eerste keer full update 
    }
    if(screen==SPEED){
      update_delay=100;
      int run_rectangle_length;
      int field=config.field;//default is in config.txt
      int bar_position=32;
      int bar_length=config.bar_length*1000/240;//default 100% length = 1852 m
      if(config.field==1){         //only switch if config.field==1 !!!
        if(((int)(Ublox.total_distance/1000000)%10==0)&(Ublox.alfa_distance/1000>1000))field=5;//indien x*10km, totale afstand laten zien  
        //if(S10.s_max_speed<(S10.display_speed[5]*0.95))field=8;//if run slower dan 95% of slowest run, show 1h result                  
        if((Ublox.alfa_distance/1000<350)&(alfa_window<100))field=3;//first 350 m after gibe  alfa screen !!
        if(Ublox.alfa_distance/1000>config.bar_length)field=4;//run longer dan 1852 m, NM scherm !! 
      }
      if(config.field==2){
        if(Ublox.run_distance/1000>config.bar_length)field=4;//if run longer dan 1852 m, NM scherm !!
      }
      if(config.field==7) {
        if((Ublox.alfa_distance/1000<350)&(alfa_window<100))field=3;//first 350 m after gibe  alfa screen !!  
        else field=7; 
      } 
      if(config.field==8) {
        field=8;
        if(Ublox.alfa_distance/1000>config.bar_length)field=4;//run longer dan 1852 m, NM scherm !! 
        //if(S10.s_max_speed>(S10.display_speed[5]*0.95))field=1;//if run faster dan 95% of slowest run, show AVG & run
        if((Ublox.alfa_distance/1000<350)&(alfa_window<100))field=3;//first 350 m after gibe  alfa screen !!  
      }       
      if(GPS_Signal_OK==true){
        display.setFont(&SansSerif_bold_96_nr);
        if((config.speed_large_font==1)&(config.field<=4)){
          display.setFont(&SansSerif_bold_84_nr);
        }                                                             //test for bigger font alfa
        if(config.speed_large_font==2){                               //test for bigger font speed
          display.setFont(&FreeSansBold75pt7b);
          display.setCursor(offset+5,115);
          display.print(gps_speed*calibration_speed,0);               //print main in large font
          display.setFont(&FreeSansBold30pt7b);
          display.print(".");
          display.println(int((gps_speed*calibration_speed-int(gps_speed*calibration_speed))*10),0); //int((x-int(x))*10)
        }
        else {
          display.setCursor(offset,120);
          display.println(gps_speed*calibration_speed,1);
        }
      }
      else{
        display.setCursor(offset,60);
        display.setFont(&FreeSansBold18pt7b);
        display.print("Low GPS signal !");
      }
      display.setCursor(offset,25);
      display.setFont(&FreeSansBold18pt7b);
      display.setTextColor(GxEPD_BLACK);  
      if(field<=2){
        if(config.speed_large_font==0){          
          display.setFont(&FreeSansBold12pt7b); 
          display.print("Run ");
          display.setFont(&FreeSansBold18pt7b);
          display.print(S10.s_max_speed*calibration_speed,1);//last 10s max from run
          display.setCursor(offset+122,24);
          display.setFont(&FreeSansBold12pt7b);
          display.print("Avg ");
          display.setFont(&FreeSansBold18pt7b);
          display.print(S10.avg_5runs*calibration_speed,1); 
        }
        if(config.speed_large_font==1){ 
          bar_position=40;//test voor bigger font, was 42
          display.setCursor(offset,36); 
          display.setFont(&SansSerif_bold_46_nr);   //Test for bigger alfa fonts
          display.print(S10.s_max_speed*calibration_speed,1);//last 10s max from run
          display.setCursor(offset+113,36);
          display.setFont(&FreeSansBold12pt7b);//A voor AVG !!!!
          display.print("A");
          display.setFont(&SansSerif_bold_46_nr);   //Test for bigger alfa fonts
          display.print(S10.avg_5runs*calibration_speed,1);
        }  
      }
      /*
      First 250m after jibe, if Window>99 m : Window and Exit
      Between 250m and 400m after jibe : Result Alfa (speed or MISS)
      Between 400m and 1852m after jibe : Actual Run + AVG
      More then 1852m : NM actual speed and NM Best speed
      */    
      if((field==3)&(config.speed_large_font==0)){
        bar_length=250*1000/240;//full bar length with Alfa = 250 meter
        if((alfa_window<99)&(Ublox.alfa_distance/1000<255)){  //Window alleen indien Window<100 en Run>350 meter !!!!&(A500.alfa_speed_max*calibration_speed<1)
          display.setFont(&FreeSansBold12pt7b);
			    display.print("Gate ");                                                                     // en nog geen geldige alfa
          display.setFont(&FreeSansBold18pt7b);   
          display.print(alfa_window,0);           
			  	display.print(" Ex ");
          display.setFont(&FreeSansBold18pt7b);
          if(alfa_exit>99)alfa_exit=99;//begrenzen alfa_exit...
            display.print(alfa_exit,0);//test functie				
          }
          else{
			      display.setFont(&FreeSansBold12pt7b);
            display.print("Alfa ");
            display.setFont(&FreeSansBold18pt7b);    
            display.print(A500.display_max_speed*calibration_speed,1);   //best Alfa from session on 500 m !! 
            if(A500.alfa_speed_max*calibration_speed>1){         //laatste alfa was geldig !!!!
              display.print("Al ");           //nieuwe alfa laatste gijp or MISSED !!!!
              display.setFont(&FreeSansBold18pt7b);
              display.print(A500.alfa_speed_max*calibration_speed,1);  //last alfa, back to zero 500 m after jibe !!
            }
            else{
              display.setFont(&FreeSansBold18pt7b);            //laatste alfa was ongeldig !!!!
              display.print("MISS");   
            }
          }
        }

        if((field==3)&(config.speed_large_font==1)){
          bar_length=250*1000/240;//full bar length with Alfa = 250 meter
          bar_position=40;//test voor bigger font 
          if((alfa_window<99)&(Ublox.alfa_distance/1000<255)){  //Window alleen indien Window<100 en Run>350 meter !!!!&(A500.alfa_speed_max*calibration_speed<1)
            display.setCursor(offset,36);
            display.setFont(&FreeSansBold12pt7b);
			      display.print("Gate");                                                                     // en nog geen geldige alfa
            display.setFont(&SansSerif_bold_46_nr);   
            display.print(alfa_window,0);
            display.setFont(&FreeSansBold12pt7b);           
			     	display.print(" Ex ");
            if(alfa_exit>99)alfa_exit=99;//begrenzen alfa_exit...
              display.setFont(&SansSerif_bold_46_nr); 
              display.print(alfa_exit,0);//test functie				
            }
            else{
              display.setCursor(offset,38);
              display.setFont(&FreeSansBold18pt7b);            
              //display.print(A500.display_max_speed*calibration_speed,1);   //best Alfa from session on 500 m !! 
              display.print("Alfa= ");
              display.setCursor(offset+110,38); 
              display.setFont(&SansSerif_bold_46_nr);        
              if(A500.alfa_speed_max*calibration_speed>1){         //laatste alfa was geldig !!!!
                display.print(A500.alfa_speed_max*calibration_speed,1);  //last alfa, back to zero 500 m after jibe !!
              }
              else{
                display.setFont(&FreeSansBold18pt7b);            //laatste alfa was ongeldig !!!!
                display.print("MISS");   
              }
            }
          } 
          if(field==4){
            if(config.speed_large_font==0){
              display.setFont(&FreeSansBold12pt7b); 
              display.print("NMa ");    //Actuele nautical mile
              display.setFont(&FreeSansBold18pt7b);
              display.print(M1852.m_max_speed*calibration_speed,1);//actueel topspeed 10s van deze run
              display.setCursor(offset+129,24);
              display.setFont(&FreeSansBold12pt7b);
              display.print("NM ");
              display.setFont(&FreeSansBold18pt7b);
              display.print(M1852.display_max_speed*calibration_speed,1);   //Snelste nautical mile van de sessie
            }
            if(config.speed_large_font==1){
              bar_position=40;//test voor bigger font 
              display.setCursor(offset,36);
              display.setFont(&FreeSansBold18pt7b); 
              display.print("NM= ");    //Actuele nautical mile
              display.setFont(&SansSerif_bold_46_nr); 
              display.print(M1852.m_max_speed*calibration_speed,1);//actueel topspeed NM van deze run
            }
          }  
          if(field==5){
            if(config.speed_large_font==1){
              bar_position=40;//test voor bigger font 
              display.setCursor(offset,36);
              display.setFont(&FreeSansBold12pt7b);
              display.print("Dist ");
              display.setFont(&SansSerif_bold_46_nr); 
              display.print(Ublox.total_distance/1000000,1);//Total distance in km, als test run_distance
              if(Ublox.total_distance/1000000>99.9){
                display.setFont(&FreeSansBold12pt7b);
                display.print(" km");
              }
            } 
            else{
              display.setCursor(offset,36);
              display.setFont(&FreeSansBold12pt7b);
              display.print("Dist ");
              display.setFont(&FreeSansBold18pt7b);
              display.print(Ublox.total_distance/1000000,1);//Total distance in km, als test run_distance
              display.setFont(&FreeSansBold12pt7b);
              display.print(" km");
            }
            Bat_level(offset+222,0);
          }
          if(field==6){
            display.setFont(&FreeSansBold12pt7b);
            display.print("2S ");
            display.setFont(&FreeSansBold18pt7b);
            display.print(S2.display_max_speed*calibration_speed,1);  //best 2s
            display.setCursor(offset+124,24);
            display.setFont(&FreeSansBold12pt7b);
            display.print("10S ");
            display.setFont(&FreeSansBold18pt7b);
            display.print(S10.display_max_speed*calibration_speed,1);  //best 10s run
          }      
          if(field==7){ 
            display.setFont(&FreeSansBold12pt7b);
            display.print(".5hA");
            display.setFont(&FreeSansBold18pt7b);
            display.print(S1800.avg_s*calibration_speed,1);   //actual average last 30 min
            display.setCursor(offset+124,24);
            display.setFont(&FreeSansBold12pt7b);
            display.print(".5hB");
            display.setFont(&FreeSansBold18pt7b);
            display.print(S1800.display_max_speed*calibration_speed,1);   //best average over 30 min
          }       
          if(field==8){
            display.setFont(&FreeSansBold12pt7b);
            display.print("1hA ");
            display.setFont(&FreeSansBold18pt7b);
            display.print(S3600.avg_s*calibration_speed,1);   //actual average last hour
            display.setCursor(offset+124,24);
            display.setFont(&FreeSansBold12pt7b);
            display.print("1hB ");
            display.setFont(&FreeSansBold18pt7b);
            display.print(S3600.display_max_speed*calibration_speed,1);   //best 3600s
          }
          int log_seconds=(millis()-start_logging_millis)/1000; //aantal seconden sinds loggen is gestart
          static int low_speed_seconds;
          if(S10.avg_s>2000){low_speed_seconds=0;}//if the speed is higher then 2000 mm/s, reset the counter
          low_speed_seconds++;
          if(low_speed_seconds>120){start_logging_millis=millis();}//bar will be reset if the 10s speed drops under 2m/s for more then 120 s !!!!
          run_rectangle_length=(Ublox.alfa_distance/bar_length);//240 pixels is volledige bar, ublox.alfa_distance zijn mm
          if(field==7){
            run_rectangle_length=log_seconds*240/1800;
            if (log_seconds>1800){start_logging_millis=millis();}
            }//30 minutes = full bar
          if(field==8){
            run_rectangle_length=log_seconds*240/3600;
            if (log_seconds>3600){start_logging_millis=millis();}
          }//60 minutes = full bar
          if(bar_length){
            display.fillRect(offset,bar_position,run_rectangle_length,8,GxEPD_BLACK);//balk voor run_distance weer te geven...          
          }
        }
        if(screen==AVG_10S){
        update_delay=1000;
        display.setFont(&FreeSansBold12pt7b);
        for(int i=9;i>4;i--){
          display.setCursor(offset,24*(10-i));
          display.print("Run ");display.print(10-i);display.print(": ");
          display.print(S10.avg_speed[i]*calibration_speed,2);display.print(" @");
          display.print(S10.time_hour[i]);display.print(":");
          display.print(S10.time_min[i]);
        }   
      }      
      if(screen==SPEED2){
        update_delay=100;
        if(GPS_Signal_OK==true){
          display.setFont(&FreeSansBold75pt7b);
        // change color when 2s speed is in top5
          if (S2.s_max_speed*calibration_speed>S2.avg_speed[5]*calibration_speed){
              display.fillScreen(GxEPD_BLACK);        
              display.setTextColor(GxEPD_WHITE);
              } 
          else {
              display.fillScreen(GxEPD_WHITE);      
              display.setTextColor(GxEPD_BLACK);
              }
          if(gps_speed*calibration_speed<10){
            display.setCursor(40,115);
          }
          else{
            display.setCursor(0,115);
             }
          display.print(gps_speed*calibration_speed,0);
          if(gps_speed*calibration_speed<100){
            display.setFont(&FreeSansBold30pt7b);
            display.print(".");
            display.println(int((gps_speed*calibration_speed-int(gps_speed*calibration_speed))*10),0); //int((x-int(x))*10)
            display.setFont(&FreeSansBold9pt7b);
            display.setCursor(160,14);
            display.print("Run ");
            display.print(S10.s_max_speed*calibration_speed,1);
            display.setCursor(160,28);
            display.print("2S ");
            display.print(S2.display_max_speed*calibration_speed,1);
          }
          Speed_in_Unit(offset);
          display.setTextColor(GxEPD_BLACK);
        }
        else{
          display.setCursor(offset,60);
          display.setFont(&FreeSansBold18pt7b);
          display.print("Low GPS signal !");
          Sats_level(offset);
        }
      }
      if(screen==TROUBLE){ 
         display.setFont(&FreeSansBold12pt7b);
         display.setCursor(offset,25);
         display.println("No GPS frames for more then 1s.... ");
         display.setCursor(offset,120);
         display.print(time_now);
      }  
      if(screen==STATS1){                        //2s,10sF,10sS, AVG
        update_delay=(config.Stat_screens_time-2)*1000;
        display.setFont(&FreeSansBold12pt7b);
        display.setCursor(offset,25);
        display.print("2l: ");
        display.setFont(&FreeSansBold18pt7b);
        display.print(S2.display_last_run*calibration_speed,1);//last 2s max from rundisplay.print(S2.avg_speed[9]*calibration_speed);   
        display.setFont(&FreeSansBold12pt7b);
        display.setCursor(120+offset%2,25);//zodat SXX niet groter wordt dan 244 pix
        display.print("2s: "); 
        display.setFont(&FreeSansBold18pt7b);
        display.print(S2.display_max_speed*calibration_speed);  //best 2s, was avg_speed[9]
        display.setFont(&FreeSansBold18pt7b);
        display.setCursor(offset,56);
        display.print("10sF: ");display.println(S10.display_max_speed*calibration_speed); //best 10s(Fast), was avg_speed[9]
        display.setCursor(offset,88);
        display.print("10sS: ");display.println(S10.display_speed[5]*calibration_speed);  //langzaamste 10s(Slow) run van de sessie
        display.setCursor(offset,120);
        display.print("AVG: ");display.println(S10.avg_5runs*calibration_speed); //average 5*10s
        Bat_level(offset+222,87);
      }
      if(screen==STATS2){                        //alfa 500m,1852m, 1800s,total_dist
        update_delay=(config.Stat_screens_time-2)*1000;
        static int toggle=0;
        display.setFont(&FreeSansBold18pt7b);
        display.setCursor(offset,24);
        display.print("Dist: ");display.print(Ublox.total_distance/1000,0);
        display.setFont(&FreeSansBold12pt7b);
        display.setCursor(202+offset%2,22);//zodat SXX niet groter wordt dan 244 pix
        display.print("S");
        display.println(ubxMessage.navPvt.numSV);
        display.setFont(&FreeSansBold18pt7b);
        display.setCursor(offset,56);
        display.print("1852m: ");display.println(M1852.display_max_speed*calibration_speed);//was avg_speed[9]
        display.setCursor(offset,88);
        if(toggle==0){
          display.print("1800S: ");display.println(S1800.display_max_speed*calibration_speed);
          display.setCursor(offset,120);
          display.print("Alfa: ");display.print(A500.avg_speed[9]*calibration_speed);   //best Alfa on 500 m 
          toggle=1;
        }
        else{
          display.print("3600S: ");display.println(S3600.display_max_speed*calibration_speed);
          display.setCursor(offset,120);
          display.print(time_now_sec);
          toggle=0;
        }
        Bat_level(offset+222,87);
      }
      if(screen==STATS3){                        //100m,250m, 500m,Alfa
        update_delay=(config.Stat_screens_time-2)*1000;
        display.setFont(&FreeSansBold18pt7b);
        display.setCursor(offset,24);
        display.print("100m: ");display.print(M100.avg_speed[9]*calibration_speed);  //best 2s, was avg_speed[9]
        display.setCursor(offset,56);
        display.print("250m: ");display.println(M250.display_max_speed*calibration_speed); //best 10s(Fast), was avg_speed[9]
        display.setCursor(offset,88);
        display.print("500m: ");display.println(M500.avg_speed[9]*calibration_speed);  //langzaamste 10s(Slow) run van de sessie
        display.setCursor(offset,120);
        display.print("Alfa: ");display.println(A500.avg_speed[9]*calibration_speed,2); //average 5*10s
        Bat_level(offset+222,87);
      }
      if(screen==STATS4){                        //10s,AVG,5 runs, update on the fly !!!
        update_delay=(config.Stat_screens_time-2)*1000;static int j=0;
        display.setCursor(offset,26);
        display.setFont(&FreeSansBold12pt7b);
        display.print("10s AVG: "); 
        display.setFont(&FreeSansBold18pt7b);
        display.println(s10.avg_5runs*calibration_speed,2); //eerste regel, avg_5runs krijgt update tijdens run !!
        for(int i=9;i>6;i--){
          display.setCursor(offset,56+(9-i)*32);
          display.setFont(&FreeSansBold12pt7b);display.print("R");display.print(10-i);display.print(" ");
          display.setFont(&FreeSansBold18pt7b);display.print(s10.display_speed[i]*calibration_speed,1);
          display.setCursor(offset+118,56+(9-i)*32);
          if(i>7){
              display.setFont(&FreeSansBold12pt7b);display.print(" R");display.print(13-i);display.print(" ");
              display.setFont(&FreeSansBold18pt7b);display.print(s10.display_speed[i-3]*calibration_speed,1);
          }
          else{
              display.print(time_now);
          }
        }
      }
      if(screen==STATS5){                        //alfa statistics
        update_delay=(config.Stat_screens_time-2)*1000;
        display.setCursor(offset,26);
        display.setFont(&FreeSansBold12pt7b);
        display.print("Last Alfa stats ! "); 
        display.setFont(&FreeSansBold18pt7b);
        for(int i=9;i>6;i--){
          display.setCursor(offset,56+(9-i)*32);
          display.setFont(&FreeSansBold12pt7b);display.print("A");display.print(10-i);display.print(" ");
          display.setFont(&FreeSansBold18pt7b);display.print(a500.avg_speed[i]*calibration_speed,1);
          display.setCursor(offset+118,56+(9-i)*32);
          if(i>7){
            display.setFont(&FreeSansBold12pt7b);display.print(" A");display.print(13-i);display.print(" ");
            display.setFont(&FreeSansBold18pt7b);display.print(a500.avg_speed[i-3]*calibration_speed,1);
          }
          else{
            display.print(time_now);
          }
        }
      }
      if(screen==STATS6){ //Simon stat screen
        Serial.println("STATS6_Simon_screen");
        update_delay=(config.Stat_screens_time-2)*1500;
        double S10avgNEW;
        if (S10.s_max_speed>S10.avg_speed[5]) S10avgNEW=(S10.avg_speed[9]+S10.avg_speed[8]+S10.avg_speed[7]+S10.avg_speed[6]+S10.s_max_speed)/5;
        else S10avgNEW=(S10.avg_speed[9]+S10.avg_speed[8]+S10.avg_speed[7]+S10.avg_speed[6]+S10.avg_speed[5])/5;
        int row1=15;
        int row=18;
        int row2=row1+row;
        int row3=row2+row;
        int row4=row3+row;
        int row5=row4+row;
        int row6=row5+row;
        int col1=0+offset;
        int col2=46+offset;
        int col3=114+offset;
        int col4=182+offset;
        int line=0;
    
        Bat_level_Simon(offset);
        Sats_level(offset);
          
        display.setCursor(offset,122);
        display.setFont(&FreeSansBold9pt7b);
        display.print(time_now); 
        display.setFont(&FreeMonoBold12pt7b);
        display.setCursor(col1,row1);
        display.print("AV:");
        display.setCursor(col1,row2);
        display.print("R1:");
        display.setCursor(col1,row3);
        display.print("R2:");
        display.setCursor(col1,row4);
        display.print("R3:");
        display.setCursor(col1,row5);
        display.print("R4:");
        display.setCursor(col1,row6);
        display.print("R5:");
        
        display.setFont(&FreeSansBold12pt7b);
        display.setCursor(col2,row1);
    
        display.println(S10avgNEW*calibration_speed,2); //Laat het gemiddelde incl de previous 10s zien mits in top5
        display.setCursor(col2,row2);
        display.println(S10.avg_speed[9]*calibration_speed,2);
        display.setCursor(col2,row3);
        display.println(S10.avg_speed[8]*calibration_speed,2);
        display.setCursor(col2,row4);
        display.println(S10.avg_speed[7]*calibration_speed,2);
        display.setCursor(col2,row5);
        display.println(S10.avg_speed[6]*calibration_speed,2);
        display.setCursor(col2,row6);
        display.println(S10.avg_speed[5]*calibration_speed,2);
    // right column
        display.setFont(&FreeMonoBold12pt7b);
        display.setCursor(col3,row1);
        display.print("2sec:");
        display.setCursor(col3,row2);
        display.print("Prv :");
        display.setCursor(col3,row3);
        display.print("Dist:");
        display.setCursor(col3,row4);
        display.print("Alp :");
        display.setCursor(col3,row5);
        display.print("500m:");
        display.setCursor(col3,row6);
        display.print("NM:");
    
        display.setFont(&FreeSansBold12pt7b);
        display.setCursor(col4,row1);
        display.println(S2.avg_speed[9]*calibration_speed,2);
        display.setCursor(col4,row2);
        display.println(S10.s_max_speed*calibration_speed,2);
        display.setCursor(col4,row3);
        display.println(Ublox.total_distance/1000000);
        display.setCursor(col4,row4);
        display.println(A500.avg_speed[9]*calibration_speed,2);
        display.setCursor(col4,row5);
        display.println(M500.display_max_speed*calibration_speed,2);
        display.setCursor(col4,row6);
        display.println(M1852.display_max_speed*calibration_speed,2);
    
    // position line of actual 10s run
        if (S10.s_max_speed*calibration_speed>S10.avg_speed[9]*calibration_speed) line=row1;
        else if (S10.s_max_speed*calibration_speed>S10.avg_speed[8]*calibration_speed) line=row2;
        else if (S10.s_max_speed*calibration_speed>S10.avg_speed[7]*calibration_speed) line=row3;
        else if (S10.s_max_speed*calibration_speed>S10.avg_speed[6]*calibration_speed) line=row4;
        else if (S10.s_max_speed*calibration_speed>S10.avg_speed[5]*calibration_speed) line=row5;
        else line=row6;
        display.fillRect(0,line+2,col3-10,2,GxEPD_BLACK);//lijn voor actuele run weer te geven...
      }
      if(screen==STATS7){ //Simon bar graph screen
        Serial.println("STATS7_Simon_bar graph");
        update_delay=(config.Stat_screens_time-2)*1500;
        int DisplayWidth=255;
        int DisplayHeight=122;
        int posX=5;
        int posY=120;
        int MaxNumberBar=NR_OF_BAR;//is 42, zie GPS_data.h
        int barLengthMax=70;
        int GraphWidth=215;//was 220
        int barWidth=3;
        int barSpace=2;
        int barPitch=barWidth+barSpace;
        static int r=0;
        int top_speed = S10.display_speed[9]*calibration_speed;
        int max_bar= int(top_speed/5+1)*5;
        if(max_bar<24)max_bar=24;
        int step=3;
        if(max_bar>45)step=5;
        int min_bar=(max_bar-step*8);
        float scale=80/(max_bar-min_bar);
      
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(0,15);
        display.println("Graph : Speed runs (10sec)");//printen top tekst
        r=run_count%MaxNumberBar+1;//laatste bar = 0 ?
        display.setFont(&FreeSansBold6pt7b);
        for(int i=0;i<9;i++){          
          display.fillRect(offset+posX,posY-(i*10),215, 1,GxEPD_BLACK);//printen hor.lijnen grafiek, van 5 tot 215
          display.setCursor(225+offset,posY-(i*10));//positie y-as legende
          display.print(min_bar+i*step);            //Printen y-as legende speed, van min_bar tot max_bar in 8 steps...
        }

        display.setCursor(0,30);
        display.print("R1-R5:");display.print(S10.display_speed[9]*calibration_speed);
        display.print(" ");display.print(S10.display_speed[8]*calibration_speed);
        display.print(" ");display.print(S10.display_speed[7]*calibration_speed);
        display.print(" ");display.print(S10.display_speed[6]*calibration_speed);
        display.print(" ");display.println(S10.display_speed[5]*calibration_speed);

        if (run_count<MaxNumberBar) {
          barWidth=max((GraphWidth-(r*barSpace))/(r),3);//(was r+1),3), laatste bar altijd 0; 
          barPitch=(barWidth)+barSpace; 
          for(int i=0;i<r;i++) {
            int barHeight=(S10.speed_run[i]*calibration_speed-min_bar)*scale; 
            display.fillRect(offset+posX+(i*barPitch),posY-barHeight,barWidth,barHeight,GxEPD_BLACK);
          }
        } else {
          barWidth=max((GraphWidth-(MaxNumberBar*barSpace))/MaxNumberBar,3);
          barPitch=(barWidth)+barSpace; 
          for(int i=0;i<MaxNumberBar;i++) {
            int barHeight=(S10.speed_run[(i+r)%NR_OF_BAR]*calibration_speed-min_bar)*scale;  //was S10.speed_run[i+r%NR_OF_BAR-42]  
            display.fillRect(offset+posX+(i*barPitch),posY-barHeight,barWidth,barHeight,GxEPD_BLACK);
          }    
        }
        #if defined(STATIC_DEBUG)
        Serial.print("Run: ");Serial.println(r);
        Serial.print("barWidth: ");Serial.println(barWidth);
        #endif
      }
      if(count%200==0){//was 200
        if(update_epaper>0) display.update();
        offset=0;
      }
      else {
        if(update_epaper==2){display.fillScreen(GxEPD_WHITE);}//test
        if(update_epaper>0) display.updateWindow(0,0,250,122,true);//was 244,122, true !!!
        delay(update_delay);//update delay function of screen to show
      }
      old_screen=screen;
      count++;    
    }
/*Eenmaal flankdetectie indien GPIO langer dan push_time gedrukt
* Ook variabele die dan long_pulse_time hoog blijft
* Ook variabele die optelt tot maw elke keer push
*/
    Button_push::Button_push(int GPIO_pin,int push_time,int long_pulse_time,int max_count){
    pinMode(GPIO_pin,INPUT_PULLUP);
    Input_pin=GPIO_pin;
    time_out_millis=push_time;
    max_pulse_time=long_pulse_time;
    max_button_count=max_count;
  }
  boolean Button_push::Button_pushed(void){
  return_value=false;
  button_status=digitalRead(Input_pin);
  if(digitalRead(Input_pin)==1) push_millis=millis();
  if(((millis()-push_millis)>time_out_millis)&(old_button_status==0)){
    Serial.print ("Class button push ");
    Serial.println (millis()-push_millis);
    if (long_pulse) button_count++;
    if(button_count>max_button_count)button_count=0;
    old_button_status=1;
    millis_10s=millis();
    return_value=true;
  }
  else return_value=false; 
  if((millis()-millis_10s)<(1000*max_pulse_time)) long_pulse=true;
  else long_pulse=false; 
  if(digitalRead(Input_pin)==1) old_button_status=0;
  return return_value;
}
