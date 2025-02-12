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
Bounce2::Button buttonDown = Bounce2::Button();
Bounce2::Button buttonUp = Bounce2::Button();
Bounce2::Button radioButtonDown = Bounce2::Button();
Bounce2::Button radioButtonUp = Bounce2::Button();
#define BUTTON_DOWN_PIN 21 // GIOP21 pin connected to down button
#define BUTTON_UP_PIN 22 // GIOP21 pin connected to up button
#define RADIO_BUTTON_DOWN_PIN 27 // GIOP27 pin connected to down button
#define RADIO_BUTTON_UP_PIN 26 // GIOP26 pin connected to up button
#define DEBOUNCE_TIME 5   // the debounce time in millisecond, increase this time if it still chatters
#define PWM_FREQUENCY 150 // the frequency that the controller is expecting
#define MIN_FREQ 150
#define MAX_FREQ 4000
#define PWM_RESOLUTION 8 // in bits
#define PWM_CHANNEL 0
#define PCNT_PIN 25 // GIP of opto-in on sailor hat for counting speed pulses
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

#define CzUpdatePeriod127501 10000
#define BinaryDeviceInstance 0x04 // Instance of 127501 switch state message
#define SwitchBankInstance 0x04   //Instance of 127502 change switch state message
#define NumberOfSwitches 8   // change to 4 for bit switch bank
// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = { 127502L,130813L,0 };

// Function prototypes
void SetN2kSwitchBankCommand(tN2kMsg& , unsigned char , tN2kBinaryStatus);


// Global Variables
uint8_t CzRelayPinMap[] = { 23, 5, 12, 13, 18, 14, 15, 36 }; // esp32 pins driving relays i.e CzRelayPinMap[0] returns the pin number of Relay 1
tN2kBinaryStatus CzBankStatus;
uint8_t CzSwitchState1 = 0;
uint8_t CzSwitchState2 = 0;
uint8_t CzMfdDisplaySyncState1 = 0;
uint8_t CzMfdDisplaySyncState2 = 0;

tNMEA2000 *nmea2000;
bool winchSwitchState = false;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// GPIO pin of 48V winch power
int switchPin = 14;


// setting PWM properties
// the GPIO number of the winch PWM
// we use the opto-out of the sailor hat
int pwmPin = 33;
int dutyCycle = 15; // a value from 0 to 255

// forward / reverse
int reversePin = 19;


int InterruptCounter, rpm;
volatile unsigned long pulseCount = 0;  // Count of PWM pulses

// final state machine to manage different
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
  digitalWrite(switchPin, LOW);
  ledcWriteTone(PWM_CHANNEL, MIN_FREQ);
  ws.textAll(getState());
}

void on_spinForward() {
  Serial.println("winch motor state: spinning FORWARD");
  // reverse pin off
  digitalWrite(switchPin, LOW);
  ledcWriteTone(PWM_CHANNEL, MAX_FREQ * dutyCycle / 255);
  digitalWrite(reversePin, HIGH);
  digitalWrite(switchPin, HIGH);
  ws.textAll(getState());
}

void on_spinBackward() {
  Serial.println("winch motor state: spinning BACKWARD");
  digitalWrite(switchPin, LOW);
  digitalWrite(reversePin, LOW);
  ledcWriteTone(PWM_CHANNEL, MAX_FREQ * dutyCycle / 255);
  digitalWrite(switchPin, HIGH);
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

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 

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
  if(var == "SPEED_PLACEHOLDER"){
    String speed = "";
    speed += "<p>PWM speed set: <span id=\"valueForPwmSlider\">" + String(dutyCycle) + "</span></p>";
    speed += String("<p><input type=\"range\" onchange=\"updateSliderPWM(") + String(pwmPin) + ", this)\" id=\"PwmSlider\" min=\"0\" max=\"255\" value=\"" + String(dutyCycle) + "\" step=\"1\" class=\"slider\"></p>";
    return speed;
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
  json["reversePin"] = reversePin;

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


void SetCZoneSwitchState127501(unsigned char DeviceInstance) {

    tN2kMsg N2kMsg;
    tN2kBinaryStatus BankStatus;
    N2kResetBinaryStatus(BankStatus);
    BankStatus = (BankStatus & CzMfdDisplaySyncState2) << 8; //
    BankStatus = BankStatus | CzMfdDisplaySyncState1;
    BankStatus = BankStatus | 0xffffffffffff0000ULL;
    SetN2kPGN127501(N2kMsg, DeviceInstance, BankStatus);
    nmea2000->SendMsg(N2kMsg);
}

void SetCZoneSwitchChangeRequest127502(unsigned char DeviceInstance, uint8_t SwitchIndex, bool ItemStatus)
{
    tN2kMsg N2kMsg;
    //N2kResetBinaryStatus(CzBankStatus);
    N2kSetStatusBinaryOnStatus(CzBankStatus, ItemStatus ? N2kOnOff_On : N2kOnOff_Off, SwitchIndex);
    //send out to other N2k switching devices on network ( pgn 127502 )
    SetN2kSwitchBankCommand(N2kMsg, SwitchBankInstance, CzBankStatus);
    nmea2000->SendMsg(N2kMsg);
}
void SetN2kSwitchBankCommand(tN2kMsg& N2kMsg, unsigned char DeviceBankInstance, tN2kBinaryStatus BankStatus)
{
    SetN2kPGN127502(N2kMsg, DeviceBankInstance, BankStatus);
}

//*****************************************************************************
// Change the state of the relay requested and broadcast change to other N2K switching devices
//*****************************************************************************

void SetChangeSwitchState(uint8_t SwitchIndex, bool ItemStatus) {

    // Set or reset the relay
    if (ItemStatus) {
        Serial.println("writing pin high");
        digitalWrite(CzRelayPinMap[SwitchIndex - 1], HIGH); // adjust SwitchIndex to CzRelayPinMap value and set or reset
    } else {
        Serial.println("writing pin low");
        digitalWrite(CzRelayPinMap[SwitchIndex - 1], LOW);
    }
    //send out change and status to other N2k devices on network
    SetCZoneSwitchState127501(BinaryDeviceInstance);
    SetCZoneSwitchChangeRequest127502(SwitchBankInstance, SwitchIndex, ItemStatus);
}

//*****************************************************************************
void ParseN2kPGN127502(const tN2kMsg& N2kMsg) {
    tN2kOnOff State;
    unsigned char ChangeIndex;
    int Index = 0;
    uint8_t DeviceBankInstance = N2kMsg.GetByte(Index);
    if (N2kMsg.PGN != 127502L || DeviceBankInstance  != BinaryDeviceInstance) return; //Nothing to see here
    uint16_t BankStatus = N2kMsg.Get2ByteUInt (Index);
    //Serial.println(BankStatus);
    for (ChangeIndex = 0; ChangeIndex < NumberOfSwitches; ChangeIndex++) {

        State = (tN2kOnOff)(BankStatus & 0x03);
        if (State != N2kOnOff_Unavailable) break; // index (0 to 7) found for switch and state
        BankStatus >>= 2;
    }
    Serial.println(ChangeIndex );
    Serial.println(State);

    switch (ChangeIndex) {

      case 0x00:  CzSwitchState1 ^= 0x01; // toggle state of the of switch bit
          CzMfdDisplaySyncState1 ^= 0x01; // toggle state of the of switch bit for MDF display sync
          SetChangeSwitchState(1, CzSwitchState1 & 0x01); // send the change out
          break;

      case 0x01:  CzSwitchState1 ^= 0x02;
          CzMfdDisplaySyncState1 ^= 0x04;
          SetChangeSwitchState(2, CzSwitchState1 & 0x02); // send the change out
          break;

      case 0x02:  CzSwitchState1 ^= 0x04;
          CzMfdDisplaySyncState1 ^= 0x10;
          SetChangeSwitchState(3, CzSwitchState1 & 0x04); // send the change out
          break;

      case 0x03:  CzSwitchState1 ^= 0x08;
          CzMfdDisplaySyncState1 ^= 0x40;
          SetChangeSwitchState(4, CzSwitchState1 & 0x08); // send the change out
          break;
          // second 4 switches 
      case 0x04:  CzSwitchState2 ^= 0x01; // state of the four switches
          CzMfdDisplaySyncState2 ^= 0x01; // for MDF display sync
          SetChangeSwitchState(5, CzSwitchState2 & 0x01); // send the change out
          break;
      case 0x05:  CzSwitchState2 ^= 0x02;
          CzMfdDisplaySyncState2 ^= 0x04;
          SetChangeSwitchState(6, CzSwitchState2 & 0x02); // send the change out
          break;
      case 0x06:  CzSwitchState2 ^= 0x04;
          CzMfdDisplaySyncState2 ^= 0x10;
          SetChangeSwitchState(7, CzSwitchState2 & 0x04); // send the change out
          break;
      case 0x07:  CzSwitchState2 ^= 0x08;
          CzMfdDisplaySyncState2 ^= 0x40;
          SetChangeSwitchState(8, CzSwitchState2 & 0x08); // send the change out
    }
}

// send periodic updates to maintain sync and as a "heatbeat" to the MFD

void SendN2k(void)
{
    static unsigned long CzUpdate127501 = millis();


    if (CzUpdate127501 + CzUpdatePeriod127501 < millis()) {
        CzUpdate127501 = millis();
        SetCZoneSwitchState127501(BinaryDeviceInstance);
    }
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

          // TODO: reactivate when parameter page is in place
          //       until then default values are used
          // if (json["switchPin"].as<int>() > 0) {
          //   switchPin = json["switchPin"].as<int>();
          //   pwmPin = json["pwmPin"].as<int>();
          //   reversePin = json["reversePin"].as<int>();
          // }

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
        dutyCycle = val;
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

  Serial.begin(115200);

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

  sprintf(convertedValue, "%d", reversePin); // Need to convert to string to display a default value.
  WiFiManagerParameter reverse_pin_num("reverse_pin", "GPIO # for forward/reverse", convertedValue, 7); // 7 == max length

  //add all your parameters here
  wm.addParameter(&switch_pin_num);
  wm.addParameter(&pwm_pin_num);
  wm.addParameter(&reverse_pin_num);

  if (forceConfig) {
    if (!wm.startConfigPortal("WifiTetris")) {//, "clock123")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  } else {
    if (!wm.autoConnect("WifiTetris")) {//, "clock123")) {
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

  // Initialize the output variables as outputs
  pinMode(reversePin, OUTPUT);

  // Set outputs to LOW
  digitalWrite(reversePin, HIGH);

  // configure PWM functionalitites
  ledcSetup(PWM_CHANNEL, MIN_FREQ, PWM_RESOLUTION);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pwmPin, PWM_CHANNEL);

  pinMode(PCNT_PIN, INPUT_PULLDOWN);

  // Initialize the output variables as outputs
  pinMode(CzRelayPinMap[0], OUTPUT);
  digitalWrite(CzRelayPinMap[0], LOW);

  // Initialize the output variables as outputs
  pinMode(CzRelayPinMap[1], OUTPUT);
  digitalWrite(CzRelayPinMap[1], LOW);

  buttonDown.attach( BUTTON_DOWN_PIN, INPUT_PULLUP ); // USE EXTERNAL PULL-UP
  buttonUp.attach( BUTTON_UP_PIN, INPUT_PULLUP ); // USE EXTERNAL PULL-UP
  // DEBOUNCE INTERVAL IN MILLISECONDS
  buttonDown.interval(DEBOUNCE_TIME); 
  buttonUp.interval(DEBOUNCE_TIME); 

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  buttonDown.setPressedState(LOW);
  buttonUp.setPressedState(LOW);

  // do the same for radio buttons
  radioButtonDown.attach( RADIO_BUTTON_DOWN_PIN, INPUT_PULLUP ); // USE EXTERNAL PULL-UP
  radioButtonUp.attach( RADIO_BUTTON_UP_PIN, INPUT_PULLUP ); // USE EXTERNAL PULL-UP
  // DEBOUNCE INTERVAL IN MILLISECONDS
  radioButtonDown.interval(DEBOUNCE_TIME); 
  radioButtonUp.interval(DEBOUNCE_TIME); 

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  radioButtonDown.setPressedState(LOW);
  radioButtonUp.setPressedState(LOW);

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

  reversePin = atoi(reverse_pin_num.getValue());
  Serial.print("GPIO # for forward/reverse: ");
  Serial.println(reversePin);

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

  // // initialize intitial switch state
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  N2kResetBinaryStatus(CzBankStatus);

  // // setup the N2k parameters
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);
  // //Set Product information
  nmea2000->SetProductInformation("00260001", 0001, "Switch Bank", "1.000 06/04/21", "My Yacht 8 Bit ");
  // // Set device information
  nmea2000->SetDeviceInformation(260001, 140, 30, 717);
  // // NMEA2000.SetForwardStream(&Serial);
  nmea2000->SetMode(tNMEA2000::N2km_ListenAndNode, 169);
  nmea2000->ExtendTransmitMessages(TransmitMessages);
  nmea2000->SetMsgHandler(ParseN2kPGN127502);
  nmea2000->Open();
  delay(200);
}

void countup() {
  InterruptCounter++;
}


void loop(){
  ws.cleanupClients();

  fsm.run();
  

  buttonDown.update();
  buttonUp.update();
  radioButtonDown.update();
  radioButtonUp.update();

  if (buttonDown.pressed() || radioButtonDown.pressed()) {
    fsm.trigger(triggers::forward);
  }
  if (buttonUp.pressed() || radioButtonUp.pressed()) {
    fsm.trigger(triggers::backward);
  }
  if (buttonDown.released() || buttonUp.released() || radioButtonDown.released() || radioButtonUp.released()) {
    fsm.trigger(triggers::stop);
  }

  nmea2000->ParseMessages();
  SendN2k();
}
