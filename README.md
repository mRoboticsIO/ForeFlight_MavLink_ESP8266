# ForeFlight_MavLink_ESP8266


This code has been tested using Arduino IDE 1.8.5 

Make sure you have installed the esp8266 tools version 2.4.0-rc2 by ESP8266 Community. 

1.-Arduino >> Preferences >> Additional Boards Manager URLs>> add this: 
 
https://github.com/esp8266/Arduino/releases/download/2.4.0-rc2/package_esp8266com_index.json

2.-Then go to: 

Arduino >> Board: “Any Arduino Board” >> Boards Manager.. >> esp8266 by ESP8266 Community >> Install 2.4.0-rc2 (or release version). 

3.-If for any reason you can’t see or use the 2.4.0-rc2, maybe is because the final version
has been already released, so try again by adding this link to Additional Boards Manager
and repeat the steps. : 
http://arduino.esp8266.com/stable/package_esp8266com_index.json

4.-Try to compile the board and upload it to your ESP8266 compatible devices. 

5.-Connect your serial input to your MavLink sources. 

6.-Connect your iPad to SSID “mRo ESP8266” and password “1234567890”. 

7.-Open ForeFlight and Enjoy! 

Send beers! 