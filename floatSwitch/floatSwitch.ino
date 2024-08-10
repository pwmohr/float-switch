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
} pumpState;

enum Status {OK, Error};
enum FloatState {BothUp, TopDown, BothDown, Invalid, Unknown};

Status levelErrStatus = OK;    // if the level drops below the lower float, that causes a levelStatus Error.
Status floatErrStatus = OK;    // if the floats are ever in the invalid configuration (top up, but bottom down), that causes a floatStatus Error.
Status pumpRunTimeErrStatus = OK;

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
  data["topFloat"] = "UNKNOWN";
  data["bottomFloat"] = "UNKNOWN";
  data["wifi"] = num_wifi_connections;
  data["status"] = "OK";

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
      data["topFloat"] = "Up";
      data["bottomFloat"] = "Up";
      break;

    case 1:
      floatState = Invalid;
      data["topFloat"] = "Invalid";
      data["bottomFloat"] = "Invalid";
      break;

    case 2:
      floatState = TopDown;
      data["topFloat"] = "Down";
      data["bottomFloat"] = "Up";
      break;

    case 3:
      floatState = BothDown;
      data["topFloat"] = "Down";
      data["bottomFloat"] = "Down";
      levelErrStatus = Error;
      break;

    default:
      floatState = Unknown;
      data["topFloat"] = "Unknown";
      data["bottomFloat"] = "Unknown";
      break;
  }
  data["wifi"] = num_wifi_connections;

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
    floatErrStatus = Error;
  }

  // any errors stop the pump
  if( (floatErrStatus == Error || levelErrStatus == Error || pumpRunTimeErrStatus == Error) && pumpState.running == true ) {
    turnPumpOff();
  }

  // if sensor is saying that the level is dangerously low, make sure the pump is off and set the error flag
  if( floatState == BothDown && pumpState.running == true ) {
    turnPumpOff();
    levelErrStatus = Error;
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
    pumpRunTimeErrStatus = Error;
  }

  // if sensor is saying that the level is high, shut the pump off
  if( pumpState.running == true && floatState == BothUp ) {
    turnPumpOff();
  }

  // if sensor is saying that the level is low and the pump isn't running, start it
  if( pumpState.running == false && floatState == TopDown ) {
    turnPumpOn();
  }
}

void turnPumpOn()
{
  #ifdef DEBUG
  Serial.println("Pump turning on.");
  #endif
  // code to turn pump on goes here
  pumpState.running = true;
  pumpState.startTime = millis();
}

void turnPumpOff()
{
  #ifdef DEBUG
  Serial.println("Pump turning off.");
  #endif
  // code to turn pump off goes here
  pumpState.running = false;
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

  const uint8_t gridPatterns[][MAX_Y][MAX_X] = {
    { // top float up
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
      {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
    },
    { // top float down
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}, 
    },
    { // bottom float up
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0}, 
    },
    { // bottom float down
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1}, 
      {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1}, 
    },
    { // wifi connected 0
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
    },
    { // wifi connected 1
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
    },
    { // wifi connected 2
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
    },
    { // wifi connected 3
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
    },
    { // wifi connected 4
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
    },
  };

  // Determine which float switch grids to use

  // enum FloatState {BothUp, TopDown, BothDown, Invalid, Unknown};
  switch( floatState ) {
    case BothUp:
      topGridIdx = 0;
      bottomGridIdx = 2;
      break;

    case TopDown:
      topGridIdx = 1;
      bottomGridIdx = 2;
      break;

    case BothDown:
      topGridIdx = 1;
      bottomGridIdx = 3;
      break;

    case Invalid:
      topGridIdx = 0;
      bottomGridIdx = 3;
      break;
    
    case Unknown:
      topGridIdx = 0;
      bottomGridIdx = 3;
      break;

    default:
      topGridIdx = 1;
      bottomGridIdx = 2;
      break;
  }

  int wifiSignalStrength = WiFi.RSSI();

  // Determine which wifi grids to use
  if( WiFi.status() != WL_CONNECTED ) {
    wifiGridIdx = 4;
  }
  else {
    if( wifiSignalStrength < -85 ) {
      wifiGridIdx = 5;
    }
    else if( wifiSignalStrength < -75 ) {
      wifiGridIdx = 6;
    }
    else if( wifiSignalStrength < -65 ) {
      wifiGridIdx = 7;
    }
    else {
      wifiGridIdx = 8;
    }
  }

  for( int i = 0; i < MAX_X; i++ ) {
    for( int j = 0; j < MAX_Y; j++ ) {
      grid[j][i] = gridPatterns[topGridIdx][j][i] + gridPatterns[bottomGridIdx][j][i] + gridPatterns[wifiGridIdx][j][i];
      if( levelErrStatus == Error ) {
        grid[0][0] = 1;
      }
      if( floatErrStatus == Error ) {
        grid[0][1] = 1;
      }
      if( pumpRunTimeErrStatus == Error ) {
        grid[0][2] = 1;
      }
    }
  }

  matrix.renderBitmap(grid, 8, 12);
}
