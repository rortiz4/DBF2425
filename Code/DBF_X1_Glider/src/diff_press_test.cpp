#include "ms4525do.h"

/* 
* An MS4525DO object
*/
bfs::Ms4525do pres;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial){}
  Wire.begin();
  Wire.setClock(100000);
  /* 
  * I2C address of 0x28, on bus 0, with a -1 to +1 PSI range
  */
  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  /* Starting communication with the pressure transducer */
  if (!pres.Begin()) {
    Serial.println("Error communicating with sensor");
    while(1){}
  }
}

void loop() {
  /* Read the sensor */
  if (pres.Read()) {
    /* Display the data */
    Serial.print(pres.pres_pa(), 6);
    Serial.print("\t");
    Serial.print(pres.die_temp_c(), 6);
    Serial.print("\n");
  }
  delay(10);
}