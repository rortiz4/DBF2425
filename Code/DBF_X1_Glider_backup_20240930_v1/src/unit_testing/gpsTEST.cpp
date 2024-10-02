#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GNSS
#include <SPI.h>
#include <SD.h> // Library for SD card handling

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// SD card chip select pin (adjust according to your setup)
const int chipSelect = 10; 
int fileCounter = 1;        // Start the file counter at 1
File gpsFile;               // File object to handle file writing


void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();
  Wire.setClock(200000);

    // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
    while (1); // Halt if SD card can't be initialized
  }
  Serial.println("SD card initialized.");

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  // Find the next available file number
  fileCounter = findNextFileNumber();
  Serial.print("Next available file number: ");
  Serial.println(fileCounter);

  // Create new file for logging
  createNewGPSFile();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
    while (1); // Halt if GNSS can't be initialized
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  //myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL | SFE_UBLOX_FILTER_NMEA_RMC); // Make sure the library is passing all NMEA messages to processNMEA
  //myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA); // Or, we can be kind to MicroNMEA and _only_ pass the GGA messages to it
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA | SFE_UBLOX_FILTER_NMEA_RMC);

  myGNSS.setNavigationFrequency(5);

  //This will pipe all NMEA sentences to the serial port so we can see them
  //myGNSS.setNMEAOutputPort(Serial);
  Serial.println("Latitude (deg), Longitude (deg), Hour, Minute, Seconds, Hundredths, Speed (ft/s), Altitude (meters) ");
    // Write header to the file
  gpsFile.println("Latitude (deg),Longitude (deg),Hour,Minute,Seconds,Hundredths,Speed (ft/s),Altitude (meters)");
  gpsFile.flush(); // Ensure the header is written
}

void loop()
{
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  if(nmea.isValid() == true)
  {
    long alt;
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();
    long speed = nmea.getSpeed();
    bool altitude = nmea.getAltitude(alt);
    uint8_t hour = nmea.getHour() - 4; // EST is 4 hours behind UTC
    uint8_t minute = nmea.getMinute();
    uint8_t seconds = nmea.getSecond();
    uint8_t hundredths = nmea.getHundredths();
    uint8_t numOfSats = nmea.getNumSatellites();
    

    // Serial.println("-------------------------------------------");

    // /*If you want to print to serial monitor use these prints*/
    // Serial.print("Number of Satellites: ");
    // Serial.println(numOfSats);
    // Serial.print("Latitude (deg): ");
    // Serial.println(latitude_mdeg / 1000000., 6); // Lat and Long are parsed in millionths of a degree so we must divide by 1,000,000
    // Serial.print("Longitude (deg): ");
    // Serial.println(longitude_mdeg / 1000000., 6);
    // Serial.print("Time (HH:MM:SS): ");
    // Serial.print(hour);
    // Serial.print(":");
    // Serial.print(minute);
    // Serial.print(":");
    // Serial.print(seconds);
    // Serial.print(":");
    // Serial.println(hundredths);
    // Serial.print("Speed (knots): ");
    // Serial.println(speed/ 1000.,3);
    // Serial.print("Speed (ft/s): ");
    // Serial.println(speed * (1.68781 / 1000.),3);
    // Serial.print("Altitude (meters): ");
    // Serial.println(alt/1000., 3);

    /*If you want to save to .csv file then use these prints*/
    //Serial.println("Latitude (deg), Longitude (deg), Hour, Minute, Seconds, Hundredths, Speed (ft/s), Altitude (meters) ");
    Serial.print(latitude_mdeg / 1000000., 6);
    Serial.print(", ");
    Serial.print(longitude_mdeg / 1000000., 6);
    Serial.print(", ");
    Serial.print(hour);
    Serial.print(", ");
    Serial.print(minute);
    Serial.print(", ");
    Serial.print(seconds);
    Serial.print(", ");
    Serial.print(hundredths);
    Serial.print(", ");
    Serial.print(speed * (1.68781 / 1000.),3);
    Serial.print(", ");
    Serial.println(alt/1000., 3);

    // Log GPS data to SD card in CSV format
    gpsFile.print(latitude_mdeg / 1000000., 6);
    gpsFile.print(", ");
    gpsFile.print(longitude_mdeg / 1000000., 6);
    gpsFile.print(", ");
    gpsFile.print(hour);
    gpsFile.print(", ");
    gpsFile.print(minute);
    gpsFile.print(", ");
    gpsFile.print(seconds);
    gpsFile.print(", ");
    gpsFile.print(hundredths);
    gpsFile.print(", ");
    gpsFile.print(speed * (1.68781 / 1000.), 3);
    gpsFile.print(", ");
    gpsFile.println(alt / 1000., 3);

    // Save the data to the SD card
    gpsFile.flush();

    nmea.clear(); // Clear the MicroNMEA storage to make sure we are getting fresh data
  }
  else
  {
    Serial.println("Waiting for fresh data");
  }

  delay(210); //Don't pound too hard on the I2C bus
}


// Function to find the next available file number by checking existing files on the SD card
int findNextFileNumber() {
  int counter = 1;
  char filename[20];

  while (true) {
    sprintf(filename, "gpsFile%d.txt", counter); // Create the filename with the counter
    if (!SD.exists(filename)) {
      break; // Stop when a filename doesn't exist (this is our next file number)
    }
    counter++;
  }
  return counter;
}

// Function to create a new file for the GPS data
void createNewGPSFile() {
  char filename[20];
  sprintf(filename, "gpsFile%d.txt", fileCounter); // Create the filename using the file counter

  gpsFile = SD.open(filename, FILE_WRITE);
  if (gpsFile) {
    Serial.print("Created file: ");
    Serial.println(filename);
  } else {
    Serial.println("Error creating file!");
    while (1); // Halt if file creation fails
  }
}

//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}
