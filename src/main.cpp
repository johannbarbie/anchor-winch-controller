#include <FS.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "SimpleFSM.h"
#include <Bounce2.h>
#define JSON_CONFIG_FILE "/config.json"
Bounce2::Button buttonDown = Bounce2::Button();
Bounce2::Button buttonUp = Bounce2::Button();
#define BUTTON_DOWN_PIN 21 // GIOP21 pin connected to down button
#define BUTTON_UP_PIN 22 // GIOP21 pin connected to up button
#define DEBOUNCE_TIME  50 // the debounce time in millisecond, increase this time if it still chatters


// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// GPIO pin of 48V winch power
int switchPin = 19;

int InterruptCounter, rpm;

SimpleFSM fsm;
void on_break();
void on_spinForward();
void on_spinBackward();

State s[] = {
  State("break",  on_break, NULL, NULL),
  State("spinForward",   on_spinForward, NULL, NULL),
  State("spinBackward",   on_spinBackward, NULL, NULL)
};

String getState() {
    StaticJsonDocument<100> json;
    int wantedpos;
    for (int i = 0; i < sizeof(s); i++) {
      if (fsm.getState() == &s[i]) {
         wantedpos = i;
         break;
      }   
    }
    json["controllerState"] = wantedpos;
    json["chainOut"] = 0;
    json["rpm"] = rpm;
    json["mainSwitch"] = digitalRead(switchPin);
    String response;
    serializeJson(json, response);
    return response;
}

void on_break() {
  Serial.println("winch motor state: BREAKING");
  ws.textAll(getState());
}

void on_spinForward() {
  Serial.println("winch motor state: spinning FORWARD");
  ws.textAll(getState());
}

void on_spinBackward() {
  Serial.println("winch motor state: spinning BACKWARD");
  ws.textAll(getState());
}

enum triggers {
  forward = 1,
  backward = 2,
  stop = 3
};

Transition transitions[] = {
  Transition(&s[0], &s[1], forward),
  Transition(&s[1], &s[0], stop),
  Transition(&s[0], &s[2], backward),
  Transition(&s[2], &s[0], stop)
};

int num_transitions = sizeof(transitions) / sizeof(Transition);

// the GPIO number of the winch PWM
int pwmPin = 25;

// setting PWM properties
int pwmFrequency= 25000;
int pwmResolution = 8;

// forward / reverse
int reversePin = 0;

// the GPIO numbers of the winch tacho
int tachoPin = 2;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 

String sliderValue = "0";
String rpmValue = "0";

//flag for saving data
bool shouldSaveConfig = false;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
  <head>
    <link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16'><text x='0' y='14'>&#2693;</text></svg>" />
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Anchor Winch Control</title>
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      .topnav {overflow: hidden;background-color: #143642;}
      h1 {font-size: 1.8rem;color: white;}
      h2{font-size: 1.5rem;font-weight: bold;color: #143642;}
      p {font-size: 1.9rem;}
      body {max-width: 400px; margin:0px auto; padding-bottom: 25px;}
      .slider { -webkit-appearance: none; margin: 14px; width: 360px; height: 25px; background: #FFD65C;
        outline: none; -webkit-transition: .2s; transition: opacity .2s;}
      .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; background: #003249; cursor: pointer;}
      .slider::-moz-range-thumb { width: 35px; height: 35px; background: #003249; cursor: pointer; } 
      button {padding: 15px 50px;font-size: 24px;text-align: center;outline: none;color: #fff;background-color: #0f8b8d;border: none;
        border-radius: 5px;-webkit-touch-callout: none;-webkit-user-select: none;-khtml-user-select: none;-moz-user-select: none;
        -ms-user-select: none;user-select: none;-webkit-tap-highlight-color: rgba(0,0,0,0);}
      button:disabled,button[disabled]{background-color: #cccccc;color: #666666;}
    </style>
  </head>
  <body>
    <div class="topnav"><h1>Anchor Winch Web Server</h1></div>
    <p><button id="buttonDown" onpointerdown="ws.send('down')" onpointerup="ws.send('stop')">Down</button></p>
    <p><button id="buttonStop" onpointerdown="ws.send('stop')">Stop</button></p>
    <p><button id="buttonUp" onpointerdown="ws.send('up')" onpointerup="ws.send('stop')">Up</button></p>
    %SWITCH_PLACEHOLDER%
    %WINCH_PLACEHOLDER%
    <p>speed measured: <span id="winchRPM">0</span></p>
    <p>chain out: <span id="chainOut">0</span></p>
    <script>
      var gateway = `ws://${window.location.hostname}/ws`;
      var ws;
      window.addEventListener('load', onLoad);
      function initWebSocket() {
        console.log('Trying to open a WebSocket connection...');
        ws = new WebSocket(gateway);
        ws.onopen    = onOpen;
        ws.onclose   = onClose;
        ws.onmessage = onMessage; // <-- add this line
      }
      function onOpen(event) {
        console.log('Connection opened');
        ws.send('getStatus');
      }
      function onClose(event) {
        console.log('Connection closed');
        setTimeout(initWebSocket, 2000);
      }
      function onMessage(event) {
        console.log(event.data);
        const state = JSON.parse(event.data);
        document.getElementById("winchRPM").textContent=state.rpm;
        document.getElementById("chainOut").textContent=state.chainOut;
        if (state.controllerState == 0) { // stop
          document.getElementById('buttonUp').disabled = false;
          document.getElementById('buttonDown').disabled = false;
        }
        if (state.controllerState == 1) { // forward
          document.getElementById('buttonUp').disabled = true;
          document.getElementById('buttonDown').disabled = false;
        }
        if (state.controllerState == 2) { // backward
          document.getElementById('buttonUp').disabled = false;
          document.getElementById('buttonDown').disabled = true;
        }
        
        const switchElem = document.getElementById("mainSwitch");
        if (switchElem.checked !== Boolean(state.mainSwitch)) {
          isLocalChange = true;
          switchElem.checked = !switchElem.checked;
          isLocalChange = false;
        }
      }
      function onLoad(event) {
        initWebSocket();
      }
      var isLocalChange = false;
      function updateSliderPWM(gpio, element) {
        var sliderValue = element.value;
        document.getElementById(`valueFor${element.id}`).innerHTML = sliderValue;
        ws.send("slider-"+sliderValue);
      }
      function toggleCheckbox(element) {
        if (!isLocalChange) {
          ws.send((element.checked) ? 'switchHigh' : 'switchLow');
        }
      }
    </script>
  </body>
</html>
)rawliteral";


String outputState(int output) {
  if(digitalRead(output)){
    return "checked";
  }
  else {
    return "";
  }
}

// Replaces placeholders with dynamic html in the control page
String processor(const String& var) {
  //Serial.println(var);
  if(var == "WINCH_PLACEHOLDER"){
    String winches = "";
    winches += "<p>PWM speed set: <span id=\"valueForPwmSlider\">" + sliderValue + "</span></p>";
    winches += String("<p><input type=\"range\" onchange=\"updateSliderPWM(") + String(pwmPin) + ", this)\" id=\"PwmSlider\" min=\"0\" max=\"255\" value=\"" + sliderValue + "\" step=\"1\" class=\"slider\"></p>";
    return winches;
  }
  if(var == "SWITCH_PLACEHOLDER"){
    String buttons = "";
    buttons += "<h4>State of Main Switch</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"mainSwitch\" " + outputState(switchPin) + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  return String();
}

void saveConfigFile() {
  Serial.println(F("Saving config"));
  StaticJsonDocument<512> json;
  json["switchPin"] = switchPin;
  json["pwmPin"] = pwmPin;
  json["pwmFrequency"] = pwmFrequency;
  json["pwmResolution"] = pwmResolution;
  json["reversePin"] = reversePin;
  json["tachoPin"] = tachoPin;

  File configFile = SPIFFS.open(JSON_CONFIG_FILE, "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }

  serializeJsonPretty(json, Serial);
  if (serializeJson(json, configFile) == 0) {
    Serial.println(F("Failed to write to file"));
  }
  configFile.close();
}

//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

bool loadConfigFile() {
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  // May need to make it begin(true) first time you are using SPIFFS
  // NOTE: This might not be a good way to do this! begin(true) reformats the spiffs
  // it will only get called if it fails to mount, which probably means it needs to be
  // formatted, but maybe dont use this if you have something important saved on spiffs
  // that can't be replaced.
  if (SPIFFS.begin(false) || SPIFFS.begin(true))
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists(JSON_CONFIG_FILE)) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open(JSON_CONFIG_FILE, "r");
      if (configFile) {
        Serial.println("opened config file");
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        serializeJsonPretty(json, Serial);
        if (!error) {
          Serial.println("\nparsed json");

          if (json["switchPin"].as<int>() > 0) {
            switchPin = json["switchPin"].as<int>();
            pwmPin = json["pwmPin"].as<int>();
            pwmFrequency = json["pwmFrequency"].as<int>();
            pwmResolution = json["pwmResolution"].as<int>();
            reversePin = json["reversePin"].as<int>();
            tachoPin = json["tachoPin"].as<int>();
          }

          return true;
        }
        else {
          Serial.println("failed to load json config");
        }
      }
    }
  }
  else {
    Serial.println("failed to mount FS");
  }
  //end read
  return false;
}

// This gets called when the config mode is launced, might
// be useful to update a display with this info.
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered Conf Mode");

  Serial.print("Config SSID: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());

  Serial.print("Config IP Address: ");
  Serial.println(WiFi.softAPIP());
}


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    char* dataStr = (char*)data;
    if (strcmp(dataStr, "down") == 0) {
      fsm.trigger(triggers::forward);
    } else if (strcmp(dataStr, "up") == 0) {
      fsm.trigger(triggers::backward);
    } else if (strcmp(dataStr, "stop") == 0) {
      fsm.trigger(triggers::stop);
    } else {
      if (strcmp(dataStr, "switchHigh") == 0) {
        digitalWrite(switchPin, HIGH);
      } else if (strcmp(dataStr, "switchLow") == 0) {
        digitalWrite(switchPin, LOW);
      } else if (strstr (dataStr, "slider-")) {
        int val = atoi( &dataStr[7] );
        Serial.printf("found slider value: %u \n", val);
        ledcWrite(0, val);
      }
      ws.textAll(getState());
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {

  bool forceConfig = false;

  bool spiffsSetup = loadConfigFile();
  if (!spiffsSetup)
  {
    Serial.println(F("Forcing config mode as there is no saved config"));
    forceConfig = true;
  }

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  // it is a good practice to make sure your code sets wifi mode how you want it.

  Serial.begin(9600);

  // Set up Final State Machine
  fsm.add(transitions, num_transitions);
  fsm.setInitialState(&s[0]);

  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  // wm.resetSettings();
  //set config save notify callback
  wm.setSaveConfigCallback(saveConfigCallback);
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wm.setAPCallback(configModeCallback);

  //--- additional Configs params ---


  // Text boxes for params
  char switch_pin_str[2];
  sprintf(switch_pin_str, "%d", switchPin); // Need to convert to string to display a default value.
  WiFiManagerParameter switch_pin_num("switch_pin", "GPIO # for switch", switch_pin_str, 2);
  Serial.print("trying to set default: ");
  Serial.println(switchPin);

  char convertedValue[6];
  sprintf(convertedValue, "%d", pwmPin); // Need to convert to string to display a default value.
  WiFiManagerParameter pwm_pin_num("pwm_pin", "GPIO # for PWM", convertedValue, 7); // 7 == max length

  sprintf(convertedValue, "%d", pwmFrequency); // Need to convert to string to display a default value.
  WiFiManagerParameter pwm_frequency_num("pwm_frequency", "Max frequency of PWM GPIO", convertedValue, 7); // 7 == max length

  sprintf(convertedValue, "%d", pwmResolution); // Need to convert to string to display a default value.
  WiFiManagerParameter pwm_resolution_num("pwm_resolution", "Resolution of PWM GPIO in bits", convertedValue, 7); // 7 == max length

  sprintf(convertedValue, "%d", reversePin); // Need to convert to string to display a default value.
  WiFiManagerParameter reverse_pin_num("reverse_pin", "GPIO # for forward/reverse", convertedValue, 7); // 7 == max length

  sprintf(convertedValue, "%d", tachoPin); // Need to convert to string to display a default value.
  WiFiManagerParameter tacho_pin_num("tacho_pin", "GPIO # for tacho", convertedValue, 7); // 7 == max length

  //add all your parameters here
  wm.addParameter(&switch_pin_num);
  wm.addParameter(&pwm_pin_num);
  wm.addParameter(&pwm_frequency_num);
  wm.addParameter(&pwm_resolution_num);
  wm.addParameter(&reverse_pin_num);
  wm.addParameter(&tacho_pin_num);

  if (forceConfig) {
    if (!wm.startConfigPortal("WifiTetris", "clock123")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  } else {
    if (!wm.autoConnect("WifiTetris", "clock123")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      // if we still have not connected restart and try all over again
      ESP.restart();
      delay(5000);
    }
  }

  // If we get here, we are connected to the WiFi

  // Initialize the output variables as outputs
  pinMode(switchPin, OUTPUT);

  // Set outputs to LOW
  digitalWrite(switchPin, LOW);

  // configure PWM functionalitites
  ledcSetup(0, pwmFrequency, pwmResolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pwmPin, 0);

  pinMode(tachoPin, INPUT);

  buttonDown.attach( BUTTON_DOWN_PIN, INPUT_PULLUP ); // USE EXTERNAL PULL-UP
  buttonUp.attach( BUTTON_UP_PIN, INPUT_PULLUP ); // USE EXTERNAL PULL-UP
  // DEBOUNCE INTERVAL IN MILLISECONDS
  buttonDown.interval(20); 
  buttonUp.interval(20); 

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  buttonDown.setPressedState(LOW);
  buttonUp.setPressedState(LOW);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Lets deal with the user config values

  //Convert the number value
  //switchPin = atoi(switch_pin_num.getValue());
  Serial.print("GPIO # for switch: ");
  Serial.println(switchPin);

  pwmPin = atoi(pwm_pin_num.getValue());
  Serial.print("GPIO # for pwm: ");
  Serial.println(pwmPin);

  pwmFrequency = atoi(pwm_frequency_num.getValue());
  Serial.print("Max Frequency of pwm GPIO: ");
  Serial.println(pwmFrequency);

  pwmResolution = atoi(pwm_resolution_num.getValue());
  Serial.print("Resolution of pwm GPIO: ");
  Serial.println(pwmResolution);

  reversePin = atoi(reverse_pin_num.getValue());
  Serial.print("GPIO # for forward/reverse: ");
  Serial.println(reversePin);

  tachoPin = atoi(tacho_pin_num.getValue());
  Serial.print("GPIO # for tacho: ");
  Serial.println(tachoPin);

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    saveConfigFile();
  }

  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  server.begin();
}

void countup() {
  InterruptCounter++;
}

void meassure(int sensorIndex) {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(tachoPin), countup, RISING);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(tachoPin));
  rpm = (InterruptCounter / 2) * 60;
  rpmValue = String(rpm);
  Serial.print("sensor: ");
  Serial.print(sensorIndex);
  Serial.print(" rpm read: ");
  Serial.println(rpm);
}


void loop(){
  ws.cleanupClients();
  // meassure(0);
  // meassure(1);
  fsm.run();

  buttonDown.update();
  buttonUp.update();

  if (buttonDown.pressed()) {
    fsm.trigger(triggers::forward);
  }
  if (buttonUp.pressed()) {
    fsm.trigger(triggers::backward);
  }
  if (buttonDown.released() || buttonUp.released()) {
    fsm.trigger(triggers::stop);
  }
}
