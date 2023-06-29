#ifndef DEFINITIONS_H
#define DEFINITIONS_H

//#define STATIC_DEBUG        //indien gps test zonder snelheid en met wifi actief
//#define DLS                  //set date on march 26 1:55, to test daylightsaving
#define GPIO12_ACTIF        //if GPIO12 is used as wake up, standard GPIO12 function is not activated !!
#define WAKE_UP_GPIO 0          //is also boot strap pin for the S3, if high@boot -> bootloader USB CDC activated
#define GPIO_NUM_xx GPIO_NUM_0   //default GPIO_NUM_39 type is no int
#define MIN_SPEED_START_LOGGING 2000        //was 2000 min speed in mm/s over 2 s alvorens start loggen naar SD 
#define TIME_DELAY_FIRST_FIX 10 //10 navpvt messages alvorens start loggen

#define EPOCH_2022 1640995200 //start of the year 2022 1640995200
#define UBLOX_TYPE_UNKNOWN 0
#define M8_9600BD 1
#define M10_9600BD 2
#define M8_38400BD 3
#define M10_38400BD 4
#define M9_9600BD 5
#define M9_38400BD 6
#define AUTO_DETECT 0xFF


//Connect the SD card to ESP pins
#define SDCARD_SS 14  //FSPICS0
#define SDCARD_CLK 3 //FSPI_CLK
#define SDCARD_MOSI 13 //FSPID
#define SDCARD_MISO 12 //FSPI_Q
// Connect the GPS RX/TX/Power to ESP pins RXD2 and TXD2
#define RXD2 18 //geel is Tx Ublox, Beitian wit is Tx
#define TXD2 8 //groen is Rx Ublox, Beitian groen is Rx
#define POWER1_ubx 16
#define POWER2_ubx 15
#define POWER1_ubx_NUM GPIO_NUM_15
#define POWER2_ubx_NUM GPIO_NUM_16
//Pin for lipo voltage
#define PIN_BAT 2 //adc pin for battery measurement ADC1_CH4
#define MINIMUM_VOLTAGE 0      // if lower then minimum_voltage, back to sleep.....
#define CALIBRATION_BAT_V 1.7 //voor proto 1
//Buildin LED pin : 
#define BUILD_IN_LED 17

#define uS_TO_S_FACTOR 1000000UL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  4000UL        /* Time ESP32 will go to sleep (in seconds, max is ?) */
#define WDT_TIMEOUT 60             //60 seconds WDT, opgelet zoeken naar ssid time-out<dan 10s !!!

#define MIN_numSV_FIRST_FIX 5     //alvorens start loggen, changed from 4 to 5 7.1/2023
#define MAX_Sacc_FIRST_FIX 2     //alvorens start loggen
#define MIN_numSV_GPS_SPEED_OK 4  //min aantal satellieten voor berekenen snelheid, anders 
#define MAX_Sacc_GPS_SPEED_OK 1   //max waarde Sacc voor berekenen snelheid, anders 0
#define MAX_GPS_SPEED_OK 40       //max snelheid in m/s voor berekenen snelheid, anders 0
#define EEPROM_SIZE 1             //use 1 byte in eeprom for saving type of ublox



#endif