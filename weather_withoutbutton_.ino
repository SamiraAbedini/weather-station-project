

#include "DHT.h"
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

#define DHTPIN 24     // digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // can be DHT11, DHT21, DHT22
#define BUTTON_PIN 22 

byte lastButtonState = LOW;

// create a DHT sensor object
DHT dht(DHTPIN, DHTTYPE);


//#define DEBUG_V  

 #include <Wire.h>  //  I2C Library

 //#include <DebugUtils.h>

 #include <MS561101BA.h>  // Library for the Sensor

 

 MS561101BA baro = MS561101BA();  // Initialize the Sensor


// run TouchScreen_Calibr_native.ino in MCUFRIEND_kbv examples to calibrate screen
// copy calibration data from serial monitor for XP, XM, YP, YM, TS_LEFT, TS_RT, TS_TOP, TS_BOT
const int XP = 7, XM = A1, YP = A2, YM = 6; //320x480 ID=0x0099
const int TS_LEFT = 903, TS_RT = 163, TS_TOP = 947, TS_BOT = 165;

MCUFRIEND_kbv tft;                                 // create display object
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300); // create touch screen object

// assign human-readable names to some common 16-bit color values:
#define WHITE    0x0000
#define YELLOW     0x001F
#define CYAN      0xF800
#define MAGENTA    0x07E0
#define RED    0x07FF
#define GREEN  0xF81F
#define BLUE   0xFFE0
#define BLACK    0xFFFF
#define DARKGRAY     0xC618
#define GRAY 0x6B0C

// thermometer graphic coordinates and size
#define thermXTop      70  // thermometer tube top center x coordinate
#define thermYTop      50  // thermometer tube top center y coordinate
#define thermHeight    190  // thermometer tube height
#define thermTubeWidth  30  // thermometer tube width
#define thermBulbRadius 30  // thermometer bulb radius

// Humidity graphic coordinates and size
#define humXTop      80  // thermometer tube top center x coordinate
#define humYTop      130  // thermometer tube top center y coordinate
#define humHeight    50  // thermometer tube height
#define humWidth  30  // thermometer tube width
#define humBulbRadius 50  // thermometer bulb


  
// touch pressure sensitivity thresholds to detect screen presses
#define MINPRESSURE 10
#define MAXPRESSURE 1000

// used to control/check if units should be displayed in C or F (makes code more readable)
#define  degC false
#define  degF true

uint16_t      tsID;                                     // touch screen driver type ID
int           pixel_x;                                  // touch screen pressed location
int           pixel_y;                                  // touch screen pressed location
uint8_t       Orientation      = 45;                     // portrait screen orientation
int           prevTempLevel    = 0;                     // store last thermometer reading level
bool          displayTempUnits = degC;                  // default to displaying in degrees C
unsigned long timeInterval     = 2000;                  // time interval between DHT readings in mS
unsigned long timeLapsed;                               // time lapsed since last timer expiry

char currentPage, selectedUnit;
// create two touch screen buttons
Adafruit_GFX_Button degC_btn;
Adafruit_GFX_Button degF_btn;

void setup(void)
{
  pinMode(BUTTON_PIN, INPUT);
  
Wire.begin();
  Serial.begin(9600);

  // initialize the DHT sensor
  dht.begin();
baro.init(MS561101BA_ADDR_CSB_LOW);   // Suppose that the CSB pin is connected to GND.

                                                                         // You'll have to check this on your breakout        

  // initialize Touch Screen
  tft.reset();
  tsID = tft.readID();                                  // find what driver is on the TFT LCD
  tft.begin(tsID);
  tft.invertDisplay(true);                              // invert screen colors if this driver requires it
  tft.setRotation(Orientation);                         // set portrait or landscape mode
  tft.fillScreen(BLACK);                                // clear the screen

 
  // draw display boxes on screen
      // header and digital temp readout box
  

  tft.setTextColor(WHITE, WHITE);        // printer header text
  tft.setTextSize(3);
  tft.setCursor(180, 50);
  tft.print("Temperature");

 currentPage = '0';
  drawThermometer();                        // draw static thermometer background graphics

}

void loop()
{
   byte buttonState = digitalRead(BUTTON_PIN);
  
  // if it is time to read from the DHT sensor, take a temp/humidity reading
  if (millis() - timeLapsed > timeInterval)  {
    // reading temperature or humidity takes about 250 milliseconds!
    // read humidity
    float humidity = dht.readHumidity();

    // read temperature: displayTempUnits true=degF false=degC
    float displayTemp = dht.readTemperature(displayTempUnits);
float temperaturee = NULL, pression = NULL;   // Initialize Temperature and Pression
while (pression == NULL)   // When Pressure is NULL (No values)

{

 

  pression = baro.getPressure(MS561101BA_OSR_4096);  // Get Pressure Values

 

}
    // Check if any reads failed
    if (isnan(humidity) || isnan(displayTemp)) {
      Serial.println(F("Failed to read from DHT sensor!"));
    }
    else {
       Serial.print(pression);   // Print Pressure Values

 Serial.println(" mbar");  // Print values in mbarmperature = baro.getTemperature(MS561101BA_OSR_4096);   // Get Temperature Values

      Serial.print(F("Humidity: "));
      Serial.print(humidity);
      Serial.print(F("%  Temperature: "));
      Serial.print(displayTemp);
      if (displayTempUnits == degC) {
        Serial.print(F("°C "));
      }
      else {
        Serial.print(F("°F "));
      }
      Serial.println();
    }


 tft.fillScreen(TFT_WHITE);
    // display temperature numerically in header box
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(7);
    tft.setCursor(180, 160);
    tft.print(displayTemp, 1);                // show temperature with 1 decimal place
    tft.print((char)247);                     // degree symbol
    if (displayTempUnits == degC)
      tft.print("C  ");
    else
      tft.print("F  ");
  drawThermometer();
  // update thermometer level graphic if temperature is within graphical range
    if ((displayTemp <= 100 ) && (displayTemp >= 0 )) {
      int curTempLevel = map(displayTemp, 0, 100, 0, (thermHeight - (thermBulbRadius)));

      // update thermometer level if height has changed
      if ((prevTempLevel - curTempLevel) != 0) {
        // draw new red level
        tft.fillRect((thermXTop - (thermTubeWidth / 2) + 6),
                     thermYTop + ((thermHeight - (thermBulbRadius) - curTempLevel)),
                     (thermTubeWidth - 12),
                     curTempLevel,
                     RED);

        // draw new white background above red level height in tube
        tft.fillRect((thermXTop - (thermTubeWidth / 2) + 3),
                     thermYTop,
                     (thermTubeWidth - 6), (thermHeight - (thermBulbRadius) - curTempLevel),
                     BLACK);
      }
      prevTempLevel = curTempLevel; // store bar height for next iteration
    }
delay(20000);
      
      tft.fillScreen(TFT_WHITE); 
       tft.setTextColor(WHITE, WHITE);        // printer header text
  tft.setTextSize(3);
  tft.setCursor(180, 50);
  tft.print("Humidity");
 tft.setTextColor(WHITE, WHITE);        // printer header text
  tft.setTextSize(10);
  tft.setCursor(250, 20);
    tft.setCursor(160, 150);
    tft.print(humidity, 1);                   // show humidity with 1 decimal place
    tft.print("%");
      
      HumidityDrop();
  delay(5000);

    
      tft.fillScreen(TFT_WHITE); 
       tft.setTextColor(WHITE, WHITE);        // printer header text
  tft.setTextSize(3);
  tft.setCursor(180, 50);
  tft.print("Pressure");
 tft.setTextColor(WHITE, WHITE);        // printer header text
  tft.setTextSize(4);
  tft.setCursor(250, 20);
    tft.setCursor(170, 150);
    tft.print(pression, 1);                   // show humidity with 1 decimal place
    tft.print("mbar");
      
      PressureGage();
      delay(5000);
     
    timeLapsed = millis();
  }

}

// draw the background graphics for the thermometer and markings
// this is only drawn once, then the temperature bar is adjusted as needed
void drawThermometer(void)
{
  // draw thermometer tube outline
  tft.fillRoundRect((thermXTop - (thermTubeWidth / 2)), thermYTop - (thermTubeWidth / 2) + 1,
                    thermTubeWidth, thermHeight, thermTubeWidth / 2, WHITE);                             // tube
  tft.fillCircle(thermXTop, (thermYTop + thermHeight), (thermBulbRadius), WHITE);                        // bottom bulb

  // fill white thermometer tube inside
  tft.fillRoundRect((thermXTop - (thermTubeWidth / 2) + 3), thermYTop - (thermTubeWidth / 2) + 5,
                    (thermTubeWidth - 6), thermHeight, (thermTubeWidth / 2) - 0.5, BLACK);               // tube
  tft.fillCircle(thermXTop, (thermYTop + thermHeight), (thermBulbRadius - 3), BLACK);                    // bottom bulb

  // fill red thermometer bulb
  tft.fillCircle(thermXTop, (thermYTop + thermHeight), (thermBulbRadius - 6), RED);

  // fill red tube to connect between lowest reading level and the bulb reservoir
  tft.fillRect((thermXTop - (thermTubeWidth / 2) + 6), thermYTop + thermHeight - thermBulbRadius, (thermTubeWidth - 12), 7, RED);

  // draw line markings beside thermometer for visual reference levels
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, thermYTop, 8, 2, BLACK);                                             // top level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.75), 5, 2, WHITE);  // 1/4 from top
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.50), 8, 2, WHITE);  // center
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.25), 5, 2, WHITE);  // 1/4 from bot
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, ((thermYTop + thermHeight) - thermBulbRadius), 8, 2, WHITE);         // bottom level

  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.95), 3, 2, WHITE);  //  "5" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.90), 3, 2, WHITE);  // "10" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.85), 3, 2, WHITE);  // "15" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.80), 3, 2, WHITE);  // "20" level

  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.70), 3, 2, WHITE);  // "30" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.65), 3, 2, WHITE);  // "35" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.60), 3, 2, WHITE);  // "40" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.55), 3, 2, WHITE);  // "45" level

  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.45), 3, 2, WHITE);  // "55" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.40), 3, 2, WHITE);  // "60" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.35), 3, 2, WHITE);  // "65" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.30), 3, 2, WHITE);  // "70" level

  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.20), 3, 2, WHITE);  // "80" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.15), 3, 2, WHITE);  // "85" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.10), 3, 2, WHITE);  // "90" level
  tft.fillRect(thermXTop + (thermTubeWidth / 2) + 1, (thermYTop + (thermHeight - thermBulbRadius) * 0.05), 3, 2, WHITE);  // "95" level

  // draw thermometer level indicator numbers
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(((thermXTop + (thermTubeWidth / 2)) + 10), (thermYTop - 5));
  tft.print("100");
  tft.setTextSize(2);
  tft.print((char)247);

  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(((thermXTop + (thermTubeWidth / 2)) + 10), (thermYTop + (thermHeight - thermBulbRadius) * 0.25) - 5);
  tft.print("75");
  tft.setTextSize(2);
  tft.print((char)247);

  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(((thermXTop + (thermTubeWidth / 2)) + 10), (thermYTop + (thermHeight - thermBulbRadius) * 0.5) - 5);
  tft.print("50");
  tft.setTextSize(2);
  tft.print((char)247);

  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(((thermXTop + (thermTubeWidth / 2)) + 10), (thermYTop + (thermHeight - thermBulbRadius) * 0.75) - 5);
  tft.print("25");
  tft.setTextSize(2);
  tft.print((char)247);

  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(((thermXTop + (thermTubeWidth / 2)) + 10), (thermYTop + thermHeight - thermBulbRadius - 5));
  tft.print("0");
  tft.setTextSize(2);
  tft.print((char)247);
}
void HumidityDrop(void)
{
   // draw thermometer tube outline
 tft.fillTriangle((humXTop), (humYTop - 70), (humXTop - 55), (humYTop + 50), (humXTop + 55), (humYTop + 50),
                         WHITE);                        // tube
  tft.fillCircle(humXTop, (humYTop + 50), (humBulbRadius+4), WHITE); 


   
  tft.fillTriangle((humXTop), (humYTop - 65), (humXTop - 53), (humYTop + 45), (humXTop + 53), (humYTop + 45),
                         BLACK);                        // tube
  tft.fillCircle(humXTop, (humYTop + 50), (humBulbRadius+2), BLACK);  
           // bottom bulb



tft.fillTriangle(humXTop, (humYTop - 60), (humXTop - 49), (humYTop + 40), (humXTop + 49), (humYTop + 40),
                         CYAN);
 tft.fillCircle(humXTop, (humYTop + 50), (humBulbRadius), CYAN);  

 
}

void PressureGage(void)
{

tft.fillCircle((humXTop+10), (humYTop + 42), (humBulbRadius+20), WHITE);  
tft.fillCircle((humXTop+10), (humYTop + 42), (humBulbRadius+18), BLACK);  
tft.fillCircle((humXTop+10), (humYTop + 42), (humBulbRadius+6), WHITE);  

tft.fillCircle((humXTop+10), (humYTop + 42), (humBulbRadius+4), BLACK);
  
}
