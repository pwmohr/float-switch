#include "WiFiS3.h"
#include <ArduinoJson.h>
#include <climits>
#include "arduino_secrets.h" 
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

#define ENABLE_WEB_SERVER
// #define DEBUG

/// Global Variables
WiFiServer webServer(80);
StaticJsonDocument<64> data;    // the object that stores JSON data
const int TOP_SWITCH = PIN_D0,
          BOTTOM_SWITCH = PIN_D1;

int num_wifi_connections = 0;
const unsigned long int PUMP_RUN_TIME_MS = (5*60000);   // 5 min

struct {
  bool running = false;
  long unsigned int startTime = 0;
  long unsigned int interval = 0;
  int animationSequenceValue = 0;
  long unsigned int animationStartTime = 0;
  const int animationDelay = 100; // ms
  bool animationRunning = false;
} pumpState;


enum FloatState {BothUp, TopDown, BothDown, Invalid, Unknown};

uint8_t errorStatus = 0;
const uint8_t levelErrStatus = (1<<0);        // if the level drops below the lower float, that causes a levelStatus Error.
const uint8_t floatErrStatus = (1<<1);        // if the floats are ever in the invalid configuration (top up, but bottom down), that causes a floatStatus Error.
const uint8_t pumpRunTimeErrStatus = (1<<2);  // if the pump runs too long before the top float comes back up, that causes a pumpRunTimeErrStatus.

FloatState floatState = Unknown;

#define MAX_Y 8
#define MAX_X 12

// Function Prototypes
void setupWebServer();
void printWifiStatus();
void processWebRequests();
void updateDisplay();
void controlPump();
void turnPumpOn();
void turnPumpOff();
 
 void setup() {

  matrix.begin();
  // matrix.renderBitmap(grid, 8, 12);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect 
  }
  // delay(1000);
  Serial.println("PLEASE WORK");

  // initialize pins as inputs
  // turn on 20k pullup resistors 
  pinMode(TOP_SWITCH, INPUT_PULLUP);
  pinMode(BOTTOM_SWITCH, INPUT_PULLUP);

  // initialize output data structure
  data["floatState"] = "UNKNOWN";
  data["wifi"] = num_wifi_connections;
  data["pump"] = "off";
  data["status"] = 0;

  // set up webserver 
  #ifdef ENABLE_WEB_SERVER
    setupWebServer();
  #endif
}

void loop() {
  // values for printing to the monitor
  unsigned int switchStatus;

  // read both switches and update the floatState
  switchStatus = digitalRead(TOP_SWITCH)<<1 | digitalRead(BOTTOM_SWITCH);

  switch( switchStatus ) {
    case 0:
      floatState = BothUp;
      data["floatState"] = "BothUp";
      break;

    case 1:
      floatState = Invalid;
      data["floatState"] = "Invalid";
      break;

    case 2:
      floatState = TopDown;
      data["floatState"] = "TopDown";
      break;

    case 3:
      floatState = BothDown;
      data["floatState"] = "BothDown";
      errorStatus = (errorStatus | levelErrStatus);
      break;

    default:
      floatState = Unknown;
      data["floatState"] = "Unknown";
      break;
  }
  data["wifi"] = num_wifi_connections;
  data["status"] = errorStatus;

  updateDisplay();
  controlPump();

  // check for connection
  #ifdef ENABLE_WEB_SERVER
  if (WiFi.status() != WL_CONNECTED) {
    setupWebServer();
  }
  
  // process web requests 
  processWebRequests();
  #endif
}

void controlPump()
{
  unsigned long int currentTime;

  // make invalid float state a latched error
  if( floatState == Invalid ) {
    errorStatus = (errorStatus | floatErrStatus);
  }

  // any errors stop the pump
  if( errorStatus != 0 && pumpState.running == true ) {
    turnPumpOff();
  }

  // if sensor is saying that the level is dangerously low, make sure the pump is off and set the error flag
  if( floatState == BothDown && pumpState.running == true ) {
    turnPumpOff();
    errorStatus = (errorStatus | levelErrStatus);
  }

  // calculate how long the pump has been running
  // if the clock has rolled over, the calculation is different, handled in the else branch
  currentTime = millis();
  if( pumpState.startTime < currentTime ) {
    pumpState.interval = currentTime - pumpState.startTime;
  }
  else {  
    pumpState.interval = ULONG_MAX - pumpState.startTime + currentTime;
  }

  // if the pump has been running more than PUMP_RUN_TIME_MS milliseconds, turn it off and set the error flag
  if( pumpState.running == true && pumpState.interval > PUMP_RUN_TIME_MS ) {
    turnPumpOff();
    errorStatus = (errorStatus | pumpRunTimeErrStatus);
  }

  // if sensor is saying that the level is high, shut the pump off
  if( pumpState.running == true && floatState == BothUp ) {
    turnPumpOff();
  }

  // if sensor is saying that the level is low and the pump isn't running and there are no latched errors, start it
  if( pumpState.running == false && floatState == TopDown && (errorStatus == 0)) {
    turnPumpOn();
  }
}

void turnPumpOn()
{
  #ifdef DEBUG
  Serial.println("Pump turning on.");
  #endif
  // TODO: code to turn pump on goes here
  pumpState.running = true;
  data["pump"] = "on";
  pumpState.startTime = millis();
}

void turnPumpOff()
{
  #ifdef DEBUG
  Serial.println("Pump turning off.");
  #endif
  // TODO: code to turn pump off goes here
  pumpState.running = false;
  data["pump"] = "off";
}

void setupWebServer()
{

  Serial.println("setting up web server");

  char ssid[] = SECRET_SSID;        // your network SSID (name)
  char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
  int keyIndex = 0;                 // your network key index number (needed only for WEP)

  int status = WL_IDLE_STATUS;

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // Blink LED_BUILTIN while waiting 10 seconds for connection:
    for( int i=0; i<10; i++ ) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(20);
        digitalWrite(LED_BUILTIN, LOW);
        delay(980);
    }
  }
  webServer.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status
  num_wifi_connections++;
}

void processWebRequests()
{
  WiFiClient client = webServer.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor

        // Check to see if the byte is a newline character
        if (c == '\n') 
        {
          // if the current line is blank, it means we got two newline characters in a row.
          // that signals the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header. In this case we are going
            // to send the data in JSON format
            serializeJson(data, client);

            // The HTTP response ends with another blank line:
            client.println();
            // we are all done with this interaction, so 
            // break out of the while loop:
            break;
          } 
          else 
          { 
            // the currentLine was not blank, so the newline character means
            // we should start building a new line
            currentLine = "";
          }
        } 
        else if (c != '\r') 
        {  
          // if we got anyting else but a carriage return character, add it to the currentLine
          currentLine += c;
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to: 
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  //print your board's IP adress
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Adress: ");
  Serial.println(ip);

  // print the received signal strength: 
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RRSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

}

void updateDisplay() {
  uint8_t grid[MAX_Y][MAX_X];
  int topGridIdx;
  int bottomGridIdx;
  int wifiGridIdx;
  typedef struct {
    uint8_t start_x;
    uint8_t start_y;
    uint8_t size_x;
    uint8_t size_y;
  } BufferPos;

  BufferPos topFloat;
  topFloat.start_x = 0;
  topFloat.start_y = 5;
  topFloat.size_x = 6;
  topFloat.size_y = 3;

  const uint8_t topFloatPixels[][topFloat.size_y][topFloat.size_x] =
  {
    {
        {1, 1, 0, 0, 0, 0}, 
        {1, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 0, 0},
    },
    {
        {0, 0, 1, 1, 0, 0}, 
        {1, 1, 1, 1, 1, 1},
        {0, 0, 1, 1, 0, 0}, 
    },
  };

  BufferPos bottomFloat;
  bottomFloat.start_x = 6;
  bottomFloat.start_y = 5;
  bottomFloat.size_x = 6;
  bottomFloat.size_y = 3;

  const uint8_t bottomFloatPixels[][bottomFloat.size_y][bottomFloat.size_x] = 
  {
    { // bottom float up
      {0, 0, 1, 1, 0, 0}, 
      {1, 1, 1, 1, 1, 1},
      {0, 0, 1, 1, 0, 0}, 
    },
    { // bottom float down
      {0, 0, 0, 0, 1, 1}, 
      {1, 1, 1, 1, 1, 1},
      {0, 0, 0, 0, 1, 1}, 
    },
  };

  BufferPos wifi;
  wifi.start_x = 8;
  wifi.start_y = 0;
  wifi.size_x = 4;
  wifi.size_y = 4;
  const uint8_t wifiPixels[][wifi.size_y][wifi.size_x] =
  {
    {
      {1, 1, 1, 1},
      {0, 1, 1, 1},
      {0, 0, 1, 1},
      {0, 0, 0, 1},
    },
    {
      {0, 0, 0, 0},
      {0, 1, 1, 1},
      {0, 0, 1, 1},
      {0, 0, 0, 1},
    },
    {
      {0, 0, 0, 0},
      {0, 0, 0, 0},
      {0, 0, 1, 1},
      {0, 0, 0, 1},
    },
    {
      {0, 0, 0, 0},
      {0, 0, 0, 0},
      {0, 0, 0, 0},
      {0, 0, 0, 1},
    },
  };

  uint8_t pumpRunningSequence[][2] = 
  {
    {5,3}, {5,2}, {6,2}, {6,3}
  };
  

  // Determine which float switch grids to use

  // enum FloatState {BothUp, TopDown, BothDown, Invalid, Unknown};
  switch( floatState ) {
    case BothUp:
      topGridIdx = 0;
      bottomGridIdx = 0;
      break;

    case TopDown:
      topGridIdx = 1;
      bottomGridIdx = 0;
      break;

    case BothDown:
      topGridIdx = 1;
      bottomGridIdx = 1;
      break;

    case Invalid:
      topGridIdx = 0;
      bottomGridIdx = 1;
      break;
    
    case Unknown:
      topGridIdx = 0;
      bottomGridIdx = 1;
      break;

    default:
      topGridIdx = 0;
      bottomGridIdx = 1;
      break;
  }

  int wifiSignalStrength = WiFi.RSSI();

  // Determine which wifi grids to use
  if( WiFi.status() != WL_CONNECTED ) {
    wifiGridIdx = -1;
  }
  else {
    if( wifiSignalStrength < -85 ) {
      wifiGridIdx = 3;
    }
    else if( wifiSignalStrength < -75 ) {
      wifiGridIdx = 2;
    }
    else if( wifiSignalStrength < -65 ) {
      wifiGridIdx = 1;
    }
    else {
      wifiGridIdx = 0;
    }
  }

  for( int i = 0; i < MAX_X; i++ ) {
    for( int j = 0; j < MAX_Y; j++ ) {
      // zero out the display buffer
      grid[j][i] = 0;
      
  
      // top float
      if( ((i >= topFloat.start_x) && (i < topFloat.start_x + topFloat.size_x)) && 
          ((j >= topFloat.start_y) && (j < topFloat.start_y + topFloat.size_y)) ) {
        grid[j][i] = topFloatPixels[topGridIdx][j-topFloat.start_y][i-topFloat.start_x];
      }

      // bottom float
      if( ((i >= bottomFloat.start_x) && (i < bottomFloat.start_x + bottomFloat.size_x)) && 
          ((j >= bottomFloat.start_y) && (j < bottomFloat.start_y + bottomFloat.size_y)) ) {
        grid[j][i] = bottomFloatPixels[bottomGridIdx][j-bottomFloat.start_y][i-bottomFloat.start_x];
      }

      // wifi strength
      if( ((i >= wifi.start_x) && (i < wifi.start_x + wifi.size_x)) && 
          ((j >= wifi.start_y) && (j < wifi.start_y + wifi.size_y)) && (wifiGridIdx >= 0) ) {
        grid[j][i] = wifiPixels[wifiGridIdx][j-wifi.start_y][i-wifi.start_x];
      }

      // error states
      if( errorStatus & levelErrStatus ) {
        grid[0][0] = 1;
      }
      if( errorStatus & floatErrStatus ) {
        grid[0][1] = 1;
      }
      if( errorStatus & pumpRunTimeErrStatus ) {
        grid[0][2] = 1;
      }

      // pump running indicator
      if( pumpState.running == true ) {
        unsigned long int currentTime = millis();
        unsigned long int animationInterval;

        if( pumpState.animationRunning == false ) {
          pumpState.animationSequenceValue = 0;
          pumpState.animationRunning = true;
          pumpState.animationStartTime = currentTime;
        }

        if( currentTime >= pumpState.animationStartTime ) {
          animationInterval = currentTime - pumpState.animationStartTime;
        }
        else {
          animationInterval = ULONG_MAX - pumpState.animationStartTime + currentTime;
        }

        if( animationInterval >= pumpState.animationDelay ) {
          pumpState.animationSequenceValue++;
          if( pumpState.animationSequenceValue >= 4 ) {
            pumpState.animationSequenceValue = 0;
          }
          pumpState.animationStartTime = currentTime;
        }
        int pumpRunning_X = pumpRunningSequence[pumpState.animationSequenceValue][0];
        int pumpRunning_Y = pumpRunningSequence[pumpState.animationSequenceValue][1];
        grid[pumpRunning_Y][pumpRunning_X] = 1;
      }
      else {
        pumpState.animationRunning = false;
      }
    }
  }
  matrix.renderBitmap(grid, 8, 12);
}

