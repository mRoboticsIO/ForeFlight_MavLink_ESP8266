extern "C" {
#include<user_interface.h> //Needed to run "wifi_softap_get_station_info();"
}

/* Set these to your desired credentials. */
const char *ssid = "mRo ESP8266";
const char *password = "1234567890";





void start_WiFi() {
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  //UDPClient.beginMulticast(WiFi.softAPIP(), broadcastAddress, 49002);

  IPAddress ip = broadcastAddress;
  IPAddress ip2 = WiFi.softAPIP();
  Serial.println(ip);
  Serial.println(ip2);

}

int lookingForClients()
{
  struct station_info *stat_info;
  struct ip_addr *IPaddress;
  IPAddress address;

  //if(lock == true) return 0; //Breaking the function if the IP has been detected. 
  
  stat_info = wifi_softap_get_station_info();

  if (stat_info != NULL) {
    Serial.print(" Client found: \r\n");
    IPaddress = &stat_info->ip;
    address = IPaddress->addr;
    Global_IP = address;
    Serial.print("\t");
    Serial.print(address);
    Serial.print("\r\n");
    stat_info = STAILQ_NEXT(stat_info, next);
   
  }
}

