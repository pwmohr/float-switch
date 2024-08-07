
#include "WiFiS3.h"
#include <ArduinoJson.h>
#include "arduino_secrets.h" 
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

#define ENABLE_WEB_SERVER
//#define DEBUG

/// Global Variables
WiFiServer webServer(80);
StaticJsonDocument<64> data;    // the object that stores JSON data
const int TOP_SWITCH = 0,
          BOTTOM_SWITCH = 1;
int num_wifi_connections = 0;

#define MAX_Y 8
#define MAX_X 12

// Function Prototypes
void setupWebServer();
void printWifiStatus();
void processWebRequests();
void updateDisplay(bool, bool, long);
 
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
  data["status"] = "UNKOWN";
  data["wifi"] = num_wifi_connections;

  // set up webserver 
  #ifdef ENABLE_WEB_SERVER
    setupWebServer();
  #endif
}

void loop() {
  
  // values for printing to the monitor
  bool topSwitchValue;
  bool bottomSwitchValue;
  bool validState;

  // read both switches
  // update data structure 
  // top switch check 
  if (digitalRead(TOP_SWITCH) == HIGH) {
    data["topFloat"] = "down";
    topSwitchValue = 1;
  } else {
    data["topFloat"] = "up";
    topSwitchValue = 0;
  }

  // bottom switch check 
  if (digitalRead(BOTTOM_SWITCH) == HIGH) {
     data["bottomFloat"] = "down";
     bottomSwitchValue = 1;
  } else {
    data["bottomFloat"] = "up";
    bottomSwitchValue = 0;
  }

  // check for valid state 
  if ((data["bottomFloat"] == "down") && (data["topFloat"] == "up")){
    data["status"] = "invalid";
    validState = 0;
  } else {
    data["status"] = "valid";
    validState = 1; 
  }
  data["wifi"] = num_wifi_connections;
  
  updateDisplay(topSwitchValue, bottomSwitchValue, WiFi.RSSI());

  #ifdef DEBUG
    Serial.print(topSwitchValue);
    Serial.print(" ");
    Serial.print(bottomSwitchValue);
    Serial.print(" ");
    Serial.println(validState);
    Serial.println();
    delay(1000);
  #endif

  // check for connection
  #ifdef ENABLE_WEB_SERVER
  if (WiFi.status() != WL_CONNECTED) {
    setupWebServer();
  }
  
  // process web requests 
  processWebRequests();
  #endif

  // LED flash for testing
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(20);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(980);
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

void updateDisplay(bool topSwitch, bool bottomSwitch, long wifiSignalStrength) {
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
  if( topSwitch ) {
    topGridIdx = 1;
  }
  else {
    topGridIdx = 0;
  }
  if( bottomSwitch ) {
    bottomGridIdx = 3;
  }
  else {
    bottomGridIdx = 2;
  }

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
    }
  }

  matrix.renderBitmap(grid, 8, 12);
}