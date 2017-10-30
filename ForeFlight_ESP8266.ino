
/*
 * By Jordi Muñoz Bardales. San Diego, CA. 10/12/2017 
 * Licensing: Do whatever you want with it. Just give me the credit and send me a cold beer. 
 * Visit my store at store.mrobotics.io. 
 */

#include <ESP8266WiFi.h>
//#include <WiFiClient.h>
//#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include "mavlink/common/mavlink.h"

void start_WiFi();
void start_feeds(void);
void heartBeat(void);
void comm_receive();
void GPS_loop(int rate);


long timer1 = 0;
long timer2 = 0;
long GPS_Timer = 0;
long Attitude_Timer = 0;


WiFiUDP UDPClient;
IPAddress broadcastAddress = (uint32_t)WiFi.softAPIP() | ~((uint32_t)WiFi.subnetMask()); //Very clever. https://www.countryipblocks.net/identifying-the-network-and-broadcast-address-of-a-subnet
IPAddress Global_IP; //

void setup() {
  delay(1000);
  Serial.begin(57600);
  Serial.println();
  Serial.print("Starting Wifi..");
  start_WiFi();
  Serial.print("Starting MavLink Feeds..");
  start_feeds();

}

void loop() {
  loop1(2000);//.5Hz
  loop2(1); //"1000Hz"
  GPS_loop(1001); //1Hz
  Attitude_loop(100); //5Hz 
}

  /***************************************************/
 /*Loops designed to run at different refresh rates*/
/*************************************************/

void loop1(int rate) {
  if ((millis() - timer1) > rate) {
    timer1 = millis();
    //Do stuff here
    heartBeat(); //Sending MavLink HeartBeat
    lookingForClients();
    //Stop stuff here
    //dumpClients();
  }
}

/***********************************************/
void loop2(int rate) {
  if ((millis() - timer2) > rate) {
    timer2 = millis();
    //Do stuff here
    comm_receive(); //<----------Receiving
    //Stop stuff here
  }
}
/***********************************************/
void Attitude_loop(int rate) {
  if ((millis() - Attitude_Timer) > rate) {
    Attitude_Timer = millis();
    //Do stuff here
    outATT();
    //Stop  stuff here
  }
}
/***********************************************/
void GPS_loop(int rate) {
  if ((millis() - GPS_Timer) > rate) {
    GPS_Timer = millis();
    //Do stuff here
    outGPS();
    //Stop stuffß here
  }
}
/************************************************/


