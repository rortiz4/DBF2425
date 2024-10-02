#include "ms4525do.h"
#define AIR_DENSITY 1.225 // In kg/m^3

/* 
* An MS4525DO object
*/
bfs::Ms4525do pres;

float pitot_airspeed(float diff_pres) {
    // Function to calculate airspeed from differential pressure
    // Note: Diff. Pressure must be in Pascals (Pa) or N/m^2
    // Output airspeed will be in m/s
    float airspeed = sqrt((2*diff_pres)/AIR_DENSITY);;
    return airspeed;
}

float corr_airspeed(float raw_airspeed) {
    // WIP. Will need calibration parameters from wind tunnel testing.
    float corrected_airspeed = raw_airspeed;
    return corrected_airspeed;
}

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
    // Save Data
    float diff_pres = pres.pres_pa();
    float temp_c = pres.die_temp_c();
    float raw_airspeed = pitot_airspeed(diff_pres);
    float corrected_airspeed = corr_airspeed(raw_airspeed);
    /* Display the data */
    Serial.print("Differential Pressure (Pa) = ");
    Serial.print(diff_pres, 6);
    Serial.print("\n");
    Serial.print("Temperature (C) = ");
    Serial.print(temp_c, 6);
    Serial.print("\n");

    Serial.print("Raw Airspeed = ");
    Serial.print(raw_airspeed, 6);
    Serial.print("\n");

    Serial.print("Corrected Airspeed = ");
    Serial.print(corrected_airspeed, 6);
    Serial.print("\n");
  }
  delay(10);
}