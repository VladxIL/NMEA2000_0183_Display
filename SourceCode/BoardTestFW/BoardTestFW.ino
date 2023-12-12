#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5 (Caution!!! Pin 2 before)
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4

/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-microsd-card-arduino/
  
  This sketch can be found at: Examples > SD(esp32) > SD_Test
*/


#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <Preferences.h>
#include <N2kMessages.h>
#include "N2kDataToNMEA0183.h"
#include <NMEA0183.h>
#include <NMEA0183LinuxStream.h>
#include <NMEA0183Messages.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Stream.h>
#include "BoatData.h"
#include "Adafruit_TSL2561_U.h"
#include <BME280I2C.h>
#include <Wire.h>

#define SCLK 18
#define MISO 19
#define MOSI 23
#define sdCS 25

#define EXD1 15
#define RXD1 16
#define TXD1 17

#define RXD2 26
#define TXD2 27

#define BUZZER_PIN 14
#define LED_PIN 13
#define VIN_A2D_PIN 35

#define ADC_Calibration_Value 5.0f // The real value depends on the true resistor values for the ADC input (100K / 27 K)
float voltage = 0;
int Counter = 0;
int MyTime = 0;
byte DispBuf[100];

// NMEA message for AIS receiving and multiplexing
tNMEA0183Msg NMEA0183Msg;
tNMEA0183 NMEA0183;

// Struct to update BoatData. See BoatData.h for content
tBoatData BoatData;

#define MAX_NMEA0183_MESSAGE_SIZE 150
Preferences preferences;             // Nonvolatile storage on ESP32 - To store LastDeviceAddress
int NodeAddress;  // To store last Node Address
// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {127489L, // Engine dynamic
                                                  0
                                                 };
const unsigned long ReceiveMessages[] PROGMEM = {/*126992L,*/ // System time
      127250L, // Heading
      127258L, // Magnetic variation
      128259UL,// Boat speed
      128267UL,// Depth
      129025UL,// Position
      129026L, // COG and SOG
      129029L, // GNSS
      130306L, // Wind
      128275UL,// Log
      127245UL,// Rudder
      129540UL,// GNSS Sat
      129033L, // time and date
      0
    };
    
tN2kDataToNMEA0183 tN2kDataToNMEA0183(&NMEA2000, 0);

// TSL2561
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
bool TSL2561_present = false;

// BME280 I2C
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool BME280I2C_present = false;


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

void PrintHex(byte * buffer, int len){
  for (int i = 0; i < len; i++){ 
    Serial.print(buffer[i], HEX);
    Serial.print(" ");

    if ((i != 0) && (i % 31) == 0)
      Serial.println();
  }
    
  Serial.println();
}

void ProccesBinMess(byte * buffer, int len){
  // ACK
  if (buffer[0] == 0x82)
    return;

  PrintHex(buffer, len);
}


void bspDisplay_CheckInput(void){
	uint8_t aData;
	static uint8_t aState = 0;
	static uint8_t MessLen = 0;
	static uint8_t MessCnt = 0;
	static uint8_t aBuffer[100];
	static uint8_t aBufferIndx = 0;

  while (Serial2.available() > 0){

    aData = Serial2.read();
		aBuffer[aBufferIndx++] = aData;

    	switch (aState) {
			case 0:
				if (aData == 0x5A){
					aState++;
					aBufferIndx = 1;
				}
				else{
    				aBufferIndx = 0;
				}
				break;

			case 1:
				if (aData == 0xA5)
					aState++;
				else
					aState = 0;
				break;

			// Mess len
			case 2:
				if (aData < 64){
					MessLen = aData;
					MessCnt = MessLen;
					aState++;
				}
				else
					aState = 0;
				break;

			case 3:
				if (--MessCnt == 0 ){
					ProccesBinMess(&aBuffer[3], MessLen);
					aState = 0;
				}
				break;

			default:
				aState = 0;
				aBufferIndx = 0;
			break;
		}
	}
}



void bspNMEA0183_CheckInput(void){
	uint8_t aData;
  static char nBuffer[255];

	static uint8_t aBufferIndx = 0;

  while (Serial1.available() > 0){

    aData = Serial1.read();
		

    if ((aData == '\r')||((aData == '\n'))||(aBufferIndx > 200)){

      // \r\n
      if (aBufferIndx < 2){
          aBufferIndx = 0;
          return;
      }

      nBuffer[aBufferIndx] == 0;

      //PrintHex((byte *)nBuffer, 64);

      Serial.print("NMEA0183--> (");
      Serial.print(aBufferIndx);
      Serial.print(")");
      Serial.println(nBuffer);

      aBufferIndx = 0;
    }else
      nBuffer[aBufferIndx++] = aData;

  }
}


#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500


//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
/*
129283: Cross track error
129033	  Time & Date
126992	  System Time
129540	  GNSS Satellites in View
129044		Datum
65362     propritery
65311
      127250L, // Heading
      127258L, // Magnetic variation
      128259UL,// Boat speed
      128267UL,// Depth
      129025UL,// Position
      129026L, // COG and SOG
      129029L, // GNSS
      130306L, // Wind
      128275UL,// Log
      127245UL,// Rudder
      0
    };
*/
    Serial.print("NMEA2000--> PGN: ");
    Serial.println(N2kMsg.PGN);

  // if ( !SendSeaSmart ) return;

//   char buf[MAX_NMEA2000_MESSAGE_SEASMART_SIZE];
//   if ( N2kToSeasmart(N2kMsg, millis(), buf, MAX_NMEA2000_MESSAGE_SEASMART_SIZE) == 0 ) 
//     return;

//  // SendBufToClients(buf);
//     Serial.print("NMEA2000--> buff: ");
//     Serial.println(buf);   

}

void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg) {

  // if ( !SendNMEA0183Conversion ) return;

  char buf[MAX_NMEA0183_MESSAGE_SIZE];
  if ( !NMEA0183Msg.GetMessage(buf, MAX_NMEA0183_MESSAGE_SIZE) ) 
    return;

  Serial.print("NMEA2000-->0183--> ");
  Serial.println(buf);


  // SendBufToClients(buf);
}

void configureTSL2561Sensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  // /* Update these values depending on what you've set above! */  
  // Serial.println("------------------------------------");
  // Serial.print  ("Gain:         "); Serial.println("Auto");
  // Serial.print  ("Timing:       "); Serial.println("13 ms");
  // Serial.println("------------------------------------");
}

void configureBME280I2CSensor(void)
{
  int i = 10;
  Wire.begin();

  // 10 times
  while(!bme.begin() &&(i-- > 0))
  {
    Serial.println("Could not find BME280 sensor!");
    return;
  }

  // switch(bme.chipModel())
  // {
  //    case BME280::ChipModel_BME280:
  //      Serial.println("Found BME280 sensor! Success.");
  //      break;
  //    case BME280::ChipModel_BMP280:
  //      Serial.println("Found BMP280 sensor! No Humidity available.");
  //      break;
  //    default:
  //      Serial.println("Found UNKNOWN sensor! Error!");
  // }
}

void printBME280Data(Stream* client){
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("BME280--> Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println("Pa");
}

// https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function/blob/master/ESP32_ADC_Read_Voltage_Accurate.ino
double ReadVoltage(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
} // Added an improved polynomial, use either, comment out as required
 
void setupADC() {
  /*
  analogReadResolution(12);             // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
                                        //  9-bit gives an ADC range of 0-511
                                        // 10-bit gives an ADC range of 0-1023
                                        // 11-bit gives an ADC range of 0-2047
                                        // 12-bit gives an ADC range of 0-4095
  analogSetCycles(8);                   // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
  analogSetSamples(1);                  // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
  analogSetClockDiv(1);                 // Set the divider for the ADC clock, default is 1, range is 1 - 255
  analogSetAttenuation(ADC_11db);       // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
  analogSetPinAttenuation(VP,ADC_11db); // Sets the input attenuation, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
                                        // ADC_0db provides no attenuation so IN/OUT = 1 / 1 an input of 3 volts remains at 3 volts before ADC measurement
                                        // ADC_2_5db provides an attenuation so that IN/OUT = 1 / 1.34 an input of 3 volts is reduced to 2.238 volts before ADC measurement
                                        // ADC_6db provides an attenuation so that IN/OUT = 1 / 2 an input of 3 volts is reduced to 1.500 volts before ADC measurement
                                        // ADC_11db provides an attenuation so that IN/OUT = 1 / 3.6 an input of 3 volts is reduced to 0.833 volts before ADC measurement
  adcAttachPin(VP);                     // Attach a pin to ADC (also clears any other analog mode that could be on), returns TRUE/FALSE result 
  adcStart(VP);                         // Starts an ADC conversion on attached pin's bus
  adcBusy(VP);                          // Check if conversion on the pin's ADC bus is currently running, returns TRUE/FALSE result 
  adcEnd(VP);                           // Get the result of the conversion (will wait if it have not finished), returns 16-bit integer result
  */
  adcAttachPin(VIN_A2D_PIN);
  analogSetClockDiv(255); // 1338mS
}

// -------------------------------------------------------------------------------------------------------------------------------------------
void setup(){
  uint8_t chipid[6];
  uint32_t id = 0;
  int i = 0;

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);

  // setupADC();

  Serial1.begin(4800, SERIAL_8N1, RXD1, TXD1);    //Hardware Serial of ESP32
  pinMode(EXD1, OUTPUT);
  digitalWrite(EXD1, HIGH);

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);    //Hardware Serial of ESP32

  DispBuf[0] = 0x5A;
  DispBuf[1] = 0xA5;
  DispBuf[2] = 0x05;
  DispBuf[3] = 0x82;

  DispBuf[4] = 0x51;
  DispBuf[5] = 0x10;

  DispBuf[6] = 0x00;
  DispBuf[7] = 0x00;

  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);

  esp_efuse_mac_get_default(chipid);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

 // Set product information
  NMEA2000.SetProductInformation("1", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "NMEA 2000 WiFi Gateway",  // Manufacturer's Model ID
                                 "1.0.2.25 (2019-07-07)",  // Manufacturer's Software version code
                                 "1.0.2.0 (2019-07-07)" // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(id, // Unique number. Use e.g. Serial number. Id is generated from MAC-Address
                                130, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);            // Show in clear text. Leave uncommented for default Actisense format.

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 32);  // Read stored last NodeAddress, default 32
  preferences.end();

  Serial.printf("NodeAddress=%d\n", NodeAddress);

  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.AttachMsgHandler(&tN2kDataToNMEA0183);       // NMEA 2000 -> NMEA 0183 conversion
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);            // Also send all NMEA2000 messages in SeaSmart format

  tN2kDataToNMEA0183.SetSendNMEA0183MessageCallback(SendNMEA0183Message);

  if (!NMEA2000.Open())
    Serial.println("ERROR! NMEA2000.Open");


  digitalWrite(BUZZER_PIN, HIGH);  
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);


  if(!SD.begin(sdCS)){
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  listDir(SD, "/", 0);
  createDir(SD, "/mydir");
  listDir(SD, "/", 0);
  removeDir(SD, "/mydir");
  listDir(SD, "/", 2);
  writeFile(SD, "/hello.txt", "Hello ");
  appendFile(SD, "/hello.txt", "World!\n");
  readFile(SD, "/hello.txt");
  deleteFile(SD, "/foo.txt");
  renameFile(SD, "/hello.txt", "/foo.txt");
  readFile(SD, "/foo.txt");
  //testFileIO(SD, "/test.txt");
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));


  /* Initialise the TSL2561 sensor */
  //use tsl.begin() to default to Wire, 
  //tsl.begin(&Wire2) directs api to use Wire2, etc.
  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }


  /* Initialise the BME280 I2C sensor */
  configureBME280I2CSensor();

  /* Setup the sensor gain and integration time */
  configureTSL2561Sensor();
}

void loop(){

  bspDisplay_CheckInput();

  bspNMEA0183_CheckInput();

  NMEA2000.ParseMessages();

  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
    NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }

  tN2kDataToNMEA0183.Update(&BoatData);

  // 1 sec ----------------------------------------------------------------
  if ((MyTime != millis()) && (millis() % 1000) == 0){
    MyTime = millis();

    //Serial.println(millis());

    DispBuf[7] = Counter++;
    Serial2.write(DispBuf, 8);

    // NMEA0183
    Serial1.println("$GPGGA,115739.00,4158.8441367,N,09147.4416929,W,4,13,0.9,255.747,M,-32.00,M,01,0000*6E");

    // 5 sec -------------------------------------------------------------
    if ((MyTime % 5000) == 0){

        Serial.println("------------- 5s --------------------");

        /* Get a new sensor event */ 
        sensors_event_t event;
        tsl.getEvent(&event);

        /* Display the results (light is measured in lux) */
        if (event.light){
          Serial.print("TSL2561--> ");
          Serial.print(event.light); 
          Serial.println(" lux");
        }
        else
        {
          /* If event.light = 0 lux the sensor is probably saturated
            and no reliable data could be generated! */
          Serial.println("TSL2561--> Sensor overload");
        }

        printBME280Data(&Serial);

        // 2.33V vhen 11.5V
        double A2D = ReadVoltage(VIN_A2D_PIN);
        Serial.print("VIN--> ");
        Serial.print(A2D * ADC_Calibration_Value); 
        Serial.println("V");
        //Serial.println((analogRead(35) * 5.12) / 1000.0);     



        
           
        Serial.println("");
    }

  }


}