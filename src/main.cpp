#include <FS.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "SimpleFSM.h"
#define JSON_CONFIG_FILE "/config.json"

SimpleFSM fsm;

void on_break() {
  Serial.println("winch motor state: BREAKING");
}

void on_spinForward() {
  Serial.println("winch motor state: spinning FORWARD");
}

void on_spinBackward() {
  Serial.println("winch motor state: spinning BACKWARD");
}

State s[] = {
  State("break",  on_break, NULL, NULL),
  State("spinForward",   on_spinForward, NULL, NULL),
  State("spinBackward",   on_spinBackward, NULL, NULL)
};

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

const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";

// GPIO pin of 48V winch power
int switchPin = 19;

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

int InterruptCounter, rpm;

String sliderValue = "0";
String rpmValue = "0";
const char* PARAM_INPUT = "value";

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
      h2 {font-size: 2.3rem;}
      p {font-size: 1.9rem;}
      body {max-width: 400px; margin:0px auto; padding-bottom: 25px;}
      .slider { -webkit-appearance: none; margin: 14px; width: 360px; height: 25px; background: #FFD65C;
        outline: none; -webkit-transition: .2s; transition: opacity .2s;}
      .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; background: #003249; cursor: pointer;}
      .slider::-moz-range-thumb { width: 35px; height: 35px; background: #003249; cursor: pointer; } 
    </style>
  </head>
  <body>
    <h1>Anchor Winch Web Server</h1>
    <p><input type="button" class="button" value="Down" onpointerdown="action(1)" onpointerup="action(3)"></p>
    <p><input type="button" class="button" value="Stop" onpointerdown="action(3)"></p>
    <p><input type="button" class="button" value="  Up  " onpointerdown="action(2)" onpointerup="action(3)"></p>
    %SWITCH_PLACEHOLDER%
    %WINCH_PLACEHOLDER%
    <p>speed measured: <span id="winchRPM">0</span></p>
    <p>chain out: <span id="chainOut">0</span></p>
    <p>controller state: <span id="contState">0</span></p>
    <script>
      var isLocalChange = false;
      function updateSliderPWM(gpio, element) {
        var sliderValue = element.value;
        document.getElementById(`valueFor${element.id}`).innerHTML = sliderValue;
        fetch("/slider?output="+gpio+"&state="+sliderValue);
      }
      function toggleCheckbox(element) {
        if (!isLocalChange) {
          fetch("/switch?output=mainSwitch&state=" + ((element.checked) ? 1 : 0));
        }
      }
      function action(value) {
        fetch("/switch?output=fsm&state=" + value);
      }
      (async () => {
        while (true) {
          const jsonResponse = await fetch('/data')
            .then(function(response) {
              return response.json();
            });
          document.getElementById("winchRPM").textContent=jsonResponse.rpm;
          document.getElementById("chainOut").textContent=jsonResponse.chainOut;
          document.getElementById("contState").textContent=jsonResponse.controllerState;
          const switchElem = document.getElementById("mainSwitch");
          if (switchElem.checked !== Boolean(jsonResponse.mainSwitch)) {
            isLocalChange = true;
            switchElem.checked = !switchElem.checked;
            isLocalChange = false;
          }
          await new Promise(resolve => setTimeout(resolve, 1000));
        }
      })();
    </script>
  </body>
</html>
)rawliteral";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

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

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  // Send a GET request to <ESP_IP>/data
  server.on("/data", HTTP_GET, [] (AsyncWebServerRequest *request) {
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
    request->send(200, "application/json", response);
  });
  // Send a GET request to <ESP_IP>/switch?output=<inputMessage1>&state=<inputMessage2>
  server.on("/switch", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String output;
    String state;
    // GET input1 value on <ESP_IP>/switch?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      output = request->getParam(PARAM_INPUT_1)->value();
      state = request->getParam(PARAM_INPUT_2)->value();
      if (output == "fsm") {
        fsm.trigger(state.toInt());
      } else {
        digitalWrite(switchPin, state.toInt());
      }
    }
    else {
      output = "No message sent";
      state = "No message sent";
    }
    Serial.print("GPIO: ");
    Serial.print(output);
    Serial.print(" - Set to: ");
    Serial.println(state);
    request->send(200, "text/plain", "OK");
  });

  // Send a GET request to <ESP_IP>/slider?output=<inputMessage1>&state=<inputMessage2>
  server.on("/slider", HTTP_GET, [] (AsyncWebServerRequest *request) {
    int pwmChannel;
    String state;
    // GET input1 value on <ESP_IP>/slider?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      pwmChannel = request->getParam(PARAM_INPUT_1)->value().toInt();
      state = request->getParam(PARAM_INPUT_2)->value();
      sliderValue = state;
      ledcWrite(0, state.toInt());
    }
    else {
      pwmChannel = 1337; // ERROR
      state = "No message sent";
    }
    Serial.print("PWM channel: ");
    Serial.print((pwmChannel > 255)? "no message sent": String(pwmChannel));
    Serial.print(" - Set to: ");
    Serial.println(state);
    request->send(200, "text/plain", "OK");
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
  // meassure(0);
  // meassure(1);
  fsm.run();
}
