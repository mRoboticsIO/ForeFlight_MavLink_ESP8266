/*
  XGPSmRo,-80.11,34.55,1200.1,359.05,55.6  1Hz
  Longitude
  Latitude
  Altitude in meters MSL
  Track-along-ground from true north
  Groundspeed in meters/sec


   XATTmRo,180.2,0.1,0.2     4-10Hz
   True Heading - float
   Pitch - degrees, float, up is positive
   Roll - degrees, float, right is positive
*/

float Global_Lon;
float Global_Lat;
float Global_Alt;
float Global_Ground_Track;
float Global_Ground_Speed;

float Global_Pitch;
float Global_Roll;
float Global_True_Heading;

//////////////////


void outATT(void) {
  String att = String("XATT mRo," + String(Global_True_Heading, 1) + "," + String(Global_Pitch, 1) + "," + String(Global_Roll, 1) + " ");
  //Serial.println(att);
  UDPClient.beginPacket(Global_IP, 49002);
  //UDPClient.beginPacketMulticast(broadcastAddress, 49002, WiFi.softAPIP(), 1);
  UDPClient.print(att);
  UDPClient.endPacket();
}

/*************************************************/

void outGPS(void) {
  String GPS = String("XGPS mRo," + String(Global_Lon / (float)100000, 4) + "," + String(Global_Lat / (float)100000, 4) + "," + String(Global_Alt, 1) + "," + String(Global_Ground_Track, 1) + "," + String(Global_Ground_Speed, 1) + " ");
  //Serial.println(GPS);
  UDPClient.beginPacket(Global_IP, 49002);
  //UDPClient.beginPacketMulticast(broadcastAddress, 49002, WiFi.softAPIP(), 1);
  UDPClient.print(GPS);
  UDPClient.endPacket();
}
