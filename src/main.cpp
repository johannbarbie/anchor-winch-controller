#include <FS.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "SimpleFSM.h"
#include <Bounce2.h>
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include <N2kMsg.h>

#define JSON_CONFIG_FILE "/config.json"

// --------------- BUTTON INSTANCES ----------------
Bounce2::Button buttonDown = Bounce2::Button();
Bounce2::Button buttonUp   = Bounce2::Button();
Bounce2::Button radioButtonDown = Bounce2::Button();
Bounce2::Button radioButtonUp   = Bounce2::Button();

// --------------- PIN DEFINITIONS ----------------
#define BUTTON_DOWN_PIN 21
#define BUTTON_UP_PIN   22
#define RADIO_BUTTON_DOWN_PIN 27
#define RADIO_BUTTON_UP_PIN   26
#define DEBOUNCE_TIME 5  // ms

// PWM frequency for motor
#define PWM_FREQUENCY 150
#define MIN_FREQ 150
#define MAX_FREQ 4000
#define PWM_RESOLUTION 8 // bits
#define PWM_CHANNEL 0

// Speed pulse counting pin
#define PCNT_PIN 25

// CAN bus pins
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

// NMEA2000 & Switch-Bank setup
#define CzUpdatePeriod127501 10000
#define BinaryDeviceInstance 0x04
#define SwitchBankInstance 0x04
#define NumberOfSwitches 8

// We will send these messages
const unsigned long TransmitMessages[] PROGMEM = { 127502L, 130813L, 0 };

// We keep pin 5 in the array to maintain the index, but treat it as a “virtual” switch.
uint8_t CzRelayPinMap[] = { 23, 5, 12, 13, 18, 14, 15, 36 };

// N2K switch statuses
tN2kBinaryStatus CzBankStatus;
uint8_t CzSwitchState1 = 0;
uint8_t CzSwitchState2 = 0;
uint8_t CzMfdDisplaySyncState1 = 0;
uint8_t CzMfdDisplaySyncState2 = 0;

// Global pointer for NMEA2000 object
tNMEA2000 *nmea2000;

// Pin controlling the 48 V motor power
int switchPin = 14;

// Winch PWM pin & dutyCycle
int pwmPin = 33;
int dutyCycle = 15;  // range 0..255

// Forward/Reverse pin
int reversePin = 19;

// For measuring speed (unused in detail but left for reference)
int InterruptCounter, rpm;
volatile unsigned long pulseCount = 0;

// --------------- WEBSOCKET & SERVER ---------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // Declare it BEFORE using it in any function

// --------------- FSM SETUP ---------------
SimpleFSM fsm;

// Forward-declare these so the FSM can reference them
void on_off();
void on_break();
void on_spinForward();
void on_spinBackward();

// We have 4 states: off, break, spinForward, spinBackward
State s[] = {
  State("off",         on_off,         NULL, NULL),
  State("break",       on_break,       NULL, NULL),
  State("spinForward", on_spinForward, NULL, NULL),
  State("spinBackward",on_spinBackward,NULL, NULL)
};

// The triggers we’ll use
enum triggers {
  toggleOn = 1,
  toggleOff,
  forward,
  backward,
  stop
};

/*
  OFF -> BREAK       : toggleOn
  BREAK -> OFF       : toggleOff
  spinForward -> OFF : toggleOff
  spinBackward -> OFF: toggleOff

  BREAK -> spinForward : forward
  spinForward -> BREAK : stop
  BREAK -> spinBackward: backward
  spinBackward -> BREAK: stop
*/
Transition transitions[] = {
  Transition(&s[0], &s[1], toggleOn),
  Transition(&s[1], &s[0], toggleOff),
  Transition(&s[2], &s[0], toggleOff),
  Transition(&s[3], &s[0], toggleOff),

  Transition(&s[1], &s[2], forward),
  Transition(&s[2], &s[1], stop),
  Transition(&s[1], &s[3], backward),
  Transition(&s[3], &s[1], stop)
};

int num_transitions = sizeof(transitions) / sizeof(Transition);

// ---------------- HELPER FUNCTIONS ----------------
String getState() {
  StaticJsonDocument<100> json;
  // figure out which state index we’re in
  int wantedpos = 0;
  for (int i = 0; i < (int)(sizeof(s)/sizeof(s[0])); i++) {
    if (fsm.getState() == &s[i]) {
       wantedpos = i;
       break;
    }
  }
  // Provide some relevant data
  json["controllerState"] = wantedpos;
  json["chainOut"] = 0; // example placeholder
  json["rpm"]     = rpm;
  json["mainSwitch"] = (wantedpos != 0); 
  // If we are in state 0 ("off"), mainSwitch = false; else true

  String response;
  serializeJson(json, response);
  return response;
}

// ---------- FSM STATE CALLBACKS ----------
void on_off() {
  Serial.println("FSM state: OFF");
  digitalWrite(switchPin, LOW);           // fully off
  ledcWriteTone(PWM_CHANNEL, MIN_FREQ);   // ensure minimal PWM
  // now we can notify all web clients
  ws.textAll(getState());
}

void on_break() {
  Serial.println("FSM state: BREAK");
  // system is on but motor is not spinning
  digitalWrite(switchPin, LOW);
  ledcWriteTone(PWM_CHANNEL, MIN_FREQ);
  ws.textAll(getState());
}

void on_spinForward() {
  Serial.println("FSM state: spinning FORWARD");
  // reverse pin off
  digitalWrite(switchPin, LOW);
  ledcWriteTone(PWM_CHANNEL, MAX_FREQ * dutyCycle / 255);
  digitalWrite(reversePin, HIGH);
  // Now engage motor power
  digitalWrite(switchPin, HIGH);
  ws.textAll(getState());
}

void on_spinBackward() {
  Serial.println("FSM state: spinning BACKWARD");
  digitalWrite(switchPin, LOW);
  digitalWrite(reversePin, LOW);
  ledcWriteTone(PWM_CHANNEL, MAX_FREQ * dutyCycle / 255);
  digitalWrite(switchPin, HIGH);
  ws.textAll(getState());
}

// --------------- HTML PAGE ---------------
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
      .slider::-webkit-slider-thumb {
        -webkit-appearance: none; appearance: none; width: 35px; height: 35px;
        background: #003249; cursor: pointer;
      }
      .slider::-moz-range-thumb {
        width: 35px; height: 35px; background: #003249; cursor: pointer;
      } 
      button {
        padding: 15px 50px; font-size: 24px; text-align: center; outline: none;
        color: #fff; background-color: #0f8b8d; border: none; border-radius: 5px;
        -webkit-touch-callout: none; -webkit-user-select: none; -khtml-user-select: none;
        -moz-user-select: none; -ms-user-select: none; user-select: none;
        -webkit-tap-highlight-color: rgba(0,0,0,0);
      }
      button:disabled,button[disabled] {
        background-color: #cccccc; color: #666666;
      }
    </style>
  </head>
  <body>
    <div class="topnav"><h1>Anchor Winch Web Server</h1></div>
    <p><button id="buttonDown" onpointerdown="ws.send('down')" onpointerup="ws.send('stop')">Down</button></p>
    <p><button id="buttonStop" onpointerdown="ws.send('stop')">Stop</button></p>
    <p><button id="buttonUp" onpointerdown="ws.send('up')" onpointerup="ws.send('stop')">Up</button></p>
    %SWITCH_PLACEHOLDER%
    %SPEED_PLACEHOLDER%
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
        ws.onmessage = onMessage;
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
        document.getElementById("winchRPM").textContent = state.rpm;
        document.getElementById("chainOut").textContent = state.chainOut;

        // 0=off, 1=break, 2=spinForward, 3=spinBackward
        if (state.controllerState == 1) {        // break
          document.getElementById('buttonUp').disabled   = false;
          document.getElementById('buttonDown').disabled = false;
        } else if (state.controllerState == 2) { // spinForward
          document.getElementById('buttonUp').disabled   = true;
          document.getElementById('buttonDown').disabled = false;
        } else if (state.controllerState == 3) { // spinBackward
          document.getElementById('buttonUp').disabled   = false;
          document.getElementById('buttonDown').disabled = true;
        } else {
          // OFF
          document.getElementById('buttonUp').disabled   = true;
          document.getElementById('buttonDown').disabled = true;
        }
        
        const switchElem = document.getElementById("mainSwitch");
        // state.mainSwitch is boolean
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
          // We signal "switchHigh" or "switchLow" to toggle OFF/ON
          ws.send((element.checked) ? 'switchHigh' : 'switchLow');
        }
      }
    </script>
  </body>
</html>
)rawliteral";

// Helper for dynamic placeholders
String outputState(int output) {
  return (digitalRead(output) ? "checked" : "");
}

// Fill placeholders in the HTML above
String processor(const String& var) {
  if(var == "SPEED_PLACEHOLDER"){
    String speed;
    speed += "<p>PWM speed set: <span id=\"valueForPwmSlider\">" + String(dutyCycle) + "</span></p>";
    speed += "<p><input type=\"range\" onchange=\"updateSliderPWM(";
    speed += String(pwmPin);
    speed += ", this)\" id=\"PwmSlider\" min=\"0\" max=\"255\" value=\"";
    speed += String(dutyCycle);
    speed += "\" step=\"1\" class=\"slider\"></p>";
    return speed;
  }
  if(var == "SWITCH_PLACEHOLDER"){
    // This is the "Main Switch" for toggling OFF/ON in the FSM
    String buttons;
    buttons  = "<h4>State of Main Switch</h4>";
    buttons += "<label class=\"switch\">";
    buttons += "<input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"mainSwitch\">";
    buttons += "<span class=\"slider\"></span>";
    buttons += "</label>";
    return buttons;
  }
  return String();
}

// ----------- SPIFFS CONFIG STORAGE -----------
bool shouldSaveConfig = false;
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void saveConfigFile() {
  Serial.println(F("Saving config"));
  StaticJsonDocument<512> json;
  json["switchPin"] = switchPin;
  json["pwmPin"]    = pwmPin;
  json["reversePin"] = reversePin;

  File configFile = SPIFFS.open(JSON_CONFIG_FILE, "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
    return;
  }
  serializeJsonPretty(json, Serial);
  if (serializeJson(json, configFile) == 0) {
    Serial.println(F("Failed to write to file"));
  }
  configFile.close();
}

bool loadConfigFile() {
  Serial.println("mounting FS...");
  if (SPIFFS.begin(false) || SPIFFS.begin(true)) {
    Serial.println("mounted file system");
    if (SPIFFS.exists(JSON_CONFIG_FILE)) {
      Serial.println("reading config file");
      File configFile = SPIFFS.open(JSON_CONFIG_FILE, "r");
      if (configFile) {
        Serial.println("opened config file");
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        serializeJsonPretty(json, Serial);
        if (!error) {
          Serial.println("\nparsed json");
          // If you want to load from the file:
          //switchPin  = json["switchPin"].as<int>();
          //pwmPin     = json["pwmPin"].as<int>();
          //reversePin = json["reversePin"].as<int>();
          return true;
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  return false;
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered Conf Mode");
  Serial.print("Config SSID: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
  Serial.print("Config IP Address: ");
  Serial.println(WiFi.softAPIP());
}

// ----------- WEBSOCKET CALLBACK -----------
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    char* dataStr = (char*)data;
    
    if (strcmp(dataStr, "getStatus") == 0) {
      // Just send current state
      ws.textAll(getState());
      return;
    }

    if (strcmp(dataStr, "down") == 0) {
      fsm.trigger(forward);
    } else if (strcmp(dataStr, "up") == 0) {
      fsm.trigger(backward);
    } else if (strcmp(dataStr, "stop") == 0) {
      fsm.trigger(stop);
    } else if (strcmp(dataStr, "switchHigh") == 0) {
      fsm.trigger(toggleOn);   // OFF -> ON
    } else if (strcmp(dataStr, "switchLow") == 0) {
      fsm.trigger(toggleOff);  // ON -> OFF
    } else if (strstr(dataStr, "slider-")) {
      int val = atoi(&dataStr[7]);
      Serial.printf("found slider value: %u \n", val);
      dutyCycle = val;
    }

    // After processing, send updated state to all clients
    ws.textAll(getState());
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n",
                    client->id(), client->remoteIP().toString().c_str());
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

// Initialize WebSocket and the main page route
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
}

// --------------- N2K SWITCH HANDLING ---------------
void SetN2kSwitchBankCommand(tN2kMsg& N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus) {
  SetN2kPGN127502(N2kMsg, DeviceBankInstance, BankStatus);
}

void SetCZoneSwitchState127501(unsigned char DeviceInstance) {
  tN2kMsg N2kMsg;
  tN2kBinaryStatus BankStatus;
  N2kResetBinaryStatus(BankStatus);

  BankStatus = (BankStatus & CzMfdDisplaySyncState2) << 8;
  BankStatus = BankStatus | CzMfdDisplaySyncState1;
  BankStatus = BankStatus | 0xffffffffffff0000ULL;

  SetN2kPGN127501(N2kMsg, DeviceInstance, BankStatus);
  nmea2000->SendMsg(N2kMsg);
}

void SetCZoneSwitchChangeRequest127502(unsigned char DeviceInstance, uint8_t SwitchIndex, bool ItemStatus) {
  tN2kMsg N2kMsg;
  N2kSetStatusBinaryOnStatus(CzBankStatus,
                             ItemStatus ? N2kOnOff_On : N2kOnOff_Off,
                             SwitchIndex);
  SetN2kSwitchBankCommand(N2kMsg, SwitchBankInstance, CzBankStatus);
  nmea2000->SendMsg(N2kMsg);
}

// If SwitchIndex == 2, skip physically toggling pin 5 (virtual switch)
void SetChangeSwitchState(uint8_t SwitchIndex, bool ItemStatus) {
  if (SwitchIndex == 2) {
    Serial.println("Switch #2 is now virtual only, no physical pin toggle.");
  } else {
    if (ItemStatus) {
      Serial.println("writing pin high");
      digitalWrite(CzRelayPinMap[SwitchIndex - 1], HIGH);
    } else {
      Serial.println("writing pin low");
      digitalWrite(CzRelayPinMap[SwitchIndex - 1], LOW);
    }
  }
  // broadcast the updated states
  SetCZoneSwitchState127501(BinaryDeviceInstance);
  SetCZoneSwitchChangeRequest127502(SwitchBankInstance, SwitchIndex, ItemStatus);
}

void ParseN2kPGN127502(const tN2kMsg& N2kMsg) {
  tN2kOnOff State;
  unsigned char ChangeIndex;
  int Index = 0;
  uint8_t DeviceBankInstance = N2kMsg.GetByte(Index);

  if (N2kMsg.PGN != 127502L || DeviceBankInstance != BinaryDeviceInstance) {
    return;
  }

  uint16_t BankStatus = N2kMsg.Get2ByteUInt(Index);
  for (ChangeIndex = 0; ChangeIndex < NumberOfSwitches; ChangeIndex++) {
    State = (tN2kOnOff)(BankStatus & 0x03);
    if (State != N2kOnOff_Unavailable) break;
    BankStatus >>= 2;
  }

  Serial.println(ChangeIndex);
  Serial.println(State);

  switch (ChangeIndex) {
    case 0x00: // Switch #1
      CzSwitchState1 ^= 0x01;
      CzMfdDisplaySyncState1 ^= 0x01;
      SetChangeSwitchState(1, CzSwitchState1 & 0x01);
      break;
    case 0x01: // Switch #2 (virtual)
      CzSwitchState1 ^= 0x02;
      CzMfdDisplaySyncState1 ^= 0x04;
      SetChangeSwitchState(2, CzSwitchState1 & 0x02);
      break;
    case 0x02:
      CzSwitchState1 ^= 0x04;
      CzMfdDisplaySyncState1 ^= 0x10;
      SetChangeSwitchState(3, CzSwitchState1 & 0x04);
      break;
    case 0x03:
      CzSwitchState1 ^= 0x08;
      CzMfdDisplaySyncState1 ^= 0x40;
      SetChangeSwitchState(4, CzSwitchState1 & 0x08);
      break;
    case 0x04:
      CzSwitchState2 ^= 0x01;
      CzMfdDisplaySyncState2 ^= 0x01;
      SetChangeSwitchState(5, CzSwitchState2 & 0x01);
      break;
    case 0x05:
      CzSwitchState2 ^= 0x02;
      CzMfdDisplaySyncState2 ^= 0x04;
      SetChangeSwitchState(6, CzSwitchState2 & 0x02);
      break;
    case 0x06:
      CzSwitchState2 ^= 0x04;
      CzMfdDisplaySyncState2 ^= 0x10;
      SetChangeSwitchState(7, CzSwitchState2 & 0x04);
      break;
    case 0x07:
      CzSwitchState2 ^= 0x08;
      CzMfdDisplaySyncState2 ^= 0x40;
      SetChangeSwitchState(8, CzSwitchState2 & 0x08);
      break;
  }
}

// Periodic heartbeat
void SendN2k(void) {
  static unsigned long CzUpdate127501 = millis();
  if (CzUpdate127501 + CzUpdatePeriod127501 < millis()) {
    CzUpdate127501 = millis();
    SetCZoneSwitchState127501(BinaryDeviceInstance);
  }
}

// ------------ SETUP & LOOP ------------
void setup() {
  Serial.begin(115200);

  bool forceConfig = false;
  bool spiffsSetup = loadConfigFile();
  if (!spiffsSetup) {
    Serial.println(F("Forcing config mode as there is no saved config"));
    forceConfig = true;
  }

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setAPCallback(configModeCallback);

  // Optional extra config parameters
  char switch_pin_str[2];
  sprintf(switch_pin_str, "%d", switchPin);
  WiFiManagerParameter switch_pin_num("switch_pin", "GPIO # for switch", switch_pin_str, 2);

  char convertedValue[7];
  sprintf(convertedValue, "%d", pwmPin);
  WiFiManagerParameter pwm_pin_num("pwm_pin", "GPIO # for PWM", convertedValue, 7);

  sprintf(convertedValue, "%d", reversePin);
  WiFiManagerParameter reverse_pin_num("reverse_pin", "GPIO # for forward/reverse", convertedValue, 7);

  wm.addParameter(&switch_pin_num);
  wm.addParameter(&pwm_pin_num);
  wm.addParameter(&reverse_pin_num);

  if (forceConfig) {
    if (!wm.startConfigPortal("WifiTetris")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      ESP.restart();
      delay(5000);
    }
  } else {
    if (!wm.autoConnect("WifiTetris")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      ESP.restart();
      delay(5000);
    }
  }

  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Possibly load param values from web manager
  // switchPin  = atoi(switch_pin_num.getValue());
  pwmPin     = atoi(pwm_pin_num.getValue());
  reversePin = atoi(reverse_pin_num.getValue());

  if (shouldSaveConfig) {
    saveConfigFile();
  }

  // ----- FSM -----
  fsm.add(transitions, num_transitions);
  fsm.setInitialState(&s[0]); // Start OFF

  // ----- I/O PINS -----
  pinMode(switchPin, OUTPUT);
  digitalWrite(switchPin, LOW);

  pinMode(reversePin, OUTPUT);
  digitalWrite(reversePin, HIGH);

  ledcSetup(PWM_CHANNEL, MIN_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pwmPin, PWM_CHANNEL);

  // Example: real pin for CzRelayPinMap[0]=23
  pinMode(CzRelayPinMap[0], OUTPUT);
  digitalWrite(CzRelayPinMap[0], LOW);
  // We do NOT do pinMode on pin 5, since that is “virtual”.

  pinMode(PCNT_PIN, INPUT_PULLDOWN);

  // local pushbuttons
  buttonDown.attach(BUTTON_DOWN_PIN, INPUT_PULLUP);
  buttonUp.attach(BUTTON_UP_PIN, INPUT_PULLUP);
  buttonDown.interval(DEBOUNCE_TIME);
  buttonUp.interval(DEBOUNCE_TIME);
  buttonDown.setPressedState(LOW);
  buttonUp.setPressedState(LOW);

  // radio remote
  radioButtonDown.attach(RADIO_BUTTON_DOWN_PIN, INPUT_PULLUP);
  radioButtonUp.attach(RADIO_BUTTON_UP_PIN, INPUT_PULLUP);
  radioButtonDown.interval(DEBOUNCE_TIME);
  radioButtonUp.interval(DEBOUNCE_TIME);
  radioButtonDown.setPressedState(LOW);
  radioButtonUp.setPressedState(LOW);

  // WebSocket init & server start
  initWebSocket();
  server.begin();

  // ----- NMEA2000 -----
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  N2kResetBinaryStatus(CzBankStatus);

  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);
  nmea2000->SetProductInformation("00260001", 0001, "Switch Bank", "1.000 06/04/21", "My Yacht 8 Bit");
  nmea2000->SetDeviceInformation(260001, 140, 30, 717);
  nmea2000->SetMode(tNMEA2000::N2km_ListenAndNode, 169);
  nmea2000->ExtendTransmitMessages(TransmitMessages);
  nmea2000->SetMsgHandler(ParseN2kPGN127502);
  nmea2000->Open();
  delay(200);
}

void loop() {
  ws.cleanupClients();
  fsm.run();

  buttonDown.update();
  buttonUp.update();
  radioButtonDown.update();
  radioButtonUp.update();

  // Pressing "down" => forward
  if (buttonDown.pressed() || radioButtonDown.pressed()) {
    fsm.trigger(forward);
  }
  // Pressing "up" => backward
  if (buttonUp.pressed() || radioButtonUp.pressed()) {
    fsm.trigger(backward);
  }
  // Releasing => stop
  if (buttonDown.released() || buttonUp.released() ||
      radioButtonDown.released() || radioButtonUp.released()) {
    fsm.trigger(stop);
  }

  nmea2000->ParseMessages();
  SendN2k();
}

