# weighbridge
An small wireless weighbridge with additional functions with HX711 and ESP8266

Please be patient: This project is strongly unter construction!

For more information (currently only in german language) look also in my blog (http://www.n8chteule.de/zentris-blog/esp8266/da-bau-ich-doch-eine-waage/)

### Remark to version 0.2.0
The measurement converter HX711 has a temperature drift.
For measurement this drift it is now implemented a connector to a DS18B20 1-wire sensor. With this sensor i will currently only dukument the context between measured weight and the housing temperature of the HX711.

The mesured weight and temperature will be now also offer via a small lokal webserver for request the data from my other project ("Erdfeuchtemessung")

