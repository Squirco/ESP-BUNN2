#include <Arduino.h>
#include <Homie.h>
#include <OneButton.h>
#include <Ticker.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Statistic.h>
#include <WS2812FX.h>
#include <ArduinoOTA.h>

// extern "C" {
// #include "user_interface.h"
// }

typedef enum {
  ACTION_SEND_SWITCH_STATE,
  ACTION_SEND_SENSOR_READINGS,
  ACTION_CONTROL_HEATER,
  ACTION_IDLE
}
machineActions;
machineActions nextAction = ACTION_IDLE; // no action when starting

bool deviceIsOnline = false;

// const int PIN_LED = 13;
const int PIN_BUTTON = 4;
const int PIN_RELAY = 5;
const int PIN_LED = 13;
const int PIN_SENSOR = 14;

//***** auto shutoff *****//
const int AUTO_SHUTOFF_TIME_M = 30;
Ticker shutoffTimer;
void autoShutoff(void);

// ***** Neopixel *****//
WS2812FX ws2812fx = WS2812FX(1, PIN_LED, NEO_RGB + NEO_KHZ800);
uint8_t ledState = 0;
Ticker ledTimer;
Ticker ledStatusControllerTimer;
void updateLed(void);
void ledStatusController(void);

//***** heater control *****//
OneWire oneWire(PIN_SENSOR);
DallasTemperature DS18B20(&oneWire);
Ticker heaterControlTimer;

const float DEFAULT_TEMP_SET_C = 96.0;
// arrays to hold device address
DeviceAddress tempSensor;
Statistic temperatureTracker;
bool temperatureIsOk;
float temperature;

//***** Relay Variables and Prototypes *****//
bool heaterOn = false;
bool heatingComplete = false;
void heaterToggleOn();
void heaterToggleOff();
void heaterToggle();
void heaterController(void);
void heaterControl(void);

//***** Button Variables and Prototypes *****//
Ticker tick;
unsigned long buttonHoldTimeStart;
bool resetTriggered = false;
void tock();
OneButton button(PIN_BUTTON, true);

HomieNode heaterNode("switch", "switch");
HomieNode temperatureNode("temperature", "temperature");
HomieNode coffeeNode("heating", "ready");

//***** ***** ********* *****//
//***** Begin Functions *****//
//***** ***** ********* *****//

void autoShutoff(){
  // ESP.restart();
  shutoffTimer.detach();
  heaterOn = false;
  heatingComplete = false;
  // if (mqttClient.connected()){
  //   mqttClient.publish(mqttCoffeeStatusState.c_str(), 1, false, "false");
  // }
}

void updateLed()
{
  ws2812fx.service();
}

void ledStatusController()
{
  if (!heaterOn) {
    if (!deviceIsOnline && ledState!=0){
      ledState = 0;
      ws2812fx.setColor(0x00FF00);
      ws2812fx.setMode(FX_MODE_STATIC);
    }
    else if (deviceIsOnline && ledState!=1){
      ledState = 1;
      ws2812fx.setColor(0x40FF00);
      ws2812fx.setMode(FX_MODE_STATIC);
    }
  }
  else if(heaterOn){
    if (!heatingComplete && ledState != 2){
      ledState = 2;
      ws2812fx.setColor(0x40FF00);
      ws2812fx.setMode(FX_MODE_BLINK);
    }
    else if (heatingComplete && ledState != 3){
      ledState = 3;
      ws2812fx.setColor(0xFF0000);
      ws2812fx.setMode(FX_MODE_STATIC);
    }
  }
}

void heaterController()
{
  if (DS18B20.isConnected(tempSensor)) {
    temperatureIsOk = true;
    temperature = DS18B20.getTempC(tempSensor);
    DS18B20.requestTemperatures();

    if (heaterOn){
      temperatureTracker.add(temperature);
      if (temperatureTracker.count() >= 10){
        float avgTemp = temperatureTracker.average();
        temperatureTracker.clear();

        if(!heatingComplete){
          if (avgTemp > DEFAULT_TEMP_SET_C - 0.5) {
            heatingComplete = true;
            shutoffTimer.attach(AUTO_SHUTOFF_TIME_M * 60, autoShutoff);
            coffeeNode.setProperty("ready").send(heatingComplete ? "true" : "false");
          }
        }
        else{
          if (abs(avgTemp-DEFAULT_TEMP_SET_C) > 2.0 ) {
            heatingComplete = false;
          }
        }
      }
    }
    temperatureNode.setProperty("temperature").send(String(temperature));
    // coffeeNode.setProperty("ready").send(heatingComplete ? "true" : "false");
  }
  else
  {
    temperatureIsOk = false;
  }
  heaterControl();
}

//***** Relay Functions *****//
void heaterControl()
{
  if (heaterOn && temperatureIsOk) {
    // Set the output
    if(temperature < DEFAULT_TEMP_SET_C)
    {
      digitalWrite(PIN_RELAY, HIGH);
    }
    else
    {
      digitalWrite(PIN_RELAY, LOW);
    }
  }
  else if (!heaterOn) {
    digitalWrite(PIN_RELAY, LOW);
  }
}

//***** Ticher and Button Functions *****//
void tock(){
	button.tick();
}

void buttonClick() {
  heaterToggle();
}

void buttonLongPressStart() {
  Homie.setIdle(false);
  buttonHoldTimeStart = millis();
}

void buttonLongPressStop(){
  Homie.setIdle(true);
  if(!resetTriggered){
    heaterToggle();
  }
}

void buttonLongPress(){
  unsigned long now = millis();
  unsigned long heldDuration = now - buttonHoldTimeStart;
  if (heldDuration >= 5000 && !resetTriggered){
    Homie.getLogger() << "RESET" << endl;
    resetTriggered = true;
  }
  if(resetTriggered){
    Homie.reset();
  }
}

void heaterToggleOn() {
  heaterOn = true;
  nextAction = ACTION_SEND_SWITCH_STATE;
}

void heaterToggleOff() {
  heaterOn = false;
  nextAction = ACTION_SEND_SWITCH_STATE;
}

void heaterToggle() {
  if(heaterOn){
    heaterToggleOff();
  }
  else{
    heaterToggleOn();
  }
}

bool heaterOnHandler(HomieRange range, String value) {
  if (value != "true" && value != "false") return false;
  heaterOn = (value == "true");
  nextAction = ACTION_SEND_SWITCH_STATE;
  return true;
}

void onHomieEvent(const HomieEvent& event) {
  switch (event.type) {
    case HomieEventType::NORMAL_MODE:
			deviceIsOnline = true;
      break;
    case HomieEventType::WIFI_DISCONNECTED:
      deviceIsOnline = false;
      break;
    case HomieEventType::MQTT_DISCONNECTED:
      deviceIsOnline = false;
      break;
  }
}

void setupHandler() {
  //***** TEMPERATURE SENSOR SETUP ******//
  DS18B20.begin();
  if (!DS18B20.getAddress(tempSensor, 0)) {
    temperatureIsOk = false;
  }
  else {
    temperatureIsOk = true;
  }
  DS18B20.setResolution(tempSensor, 12);
  DS18B20.setWaitForConversion(false);
  temperatureTracker.clear();
  heaterControlTimer.attach(1, heaterController);

  heaterOn = digitalRead(PIN_RELAY) == HIGH;
  heaterNode.setProperty("on").send(heaterOn ? "true" : "false");
  heaterNode.advertise("on").settable(heaterOnHandler);

  temperatureNode.setProperty("unit").send("°C");
  temperatureNode.advertise("unit");
  temperatureNode.advertise("temperature");

  coffeeNode.setProperty("ready").send(heatingComplete ? "true" : "false");

  //***** ARDUINO OTA *****//
  ArduinoOTA.onError([](ota_error_t error) { ESP.restart(); });
  ArduinoOTA.begin();
}

void loopHandler(){
  if (nextAction == ACTION_IDLE){
  }
  else if (nextAction == ACTION_SEND_SWITCH_STATE){
    heaterNode.setProperty("on").send(heaterOn ? "true" : "false");
    nextAction = ACTION_IDLE;
  }
  ws2812fx.service();
  ArduinoOTA.handle();
}

void setup() {
  //***** IO Setup ******//
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  //***** NEOPIXEL SETUP ******//
  ws2812fx.init();
  ws2812fx.setBrightness(255);
  ws2812fx.setSpeed(200);
  ws2812fx.setColor(0x00FF00);
  ws2812fx.setMode(FX_MODE_STATIC);
  ws2812fx.start();
  ledStatusControllerTimer.attach_ms(100,ledStatusController);
  // ledTimer.attach_ms(10,updateLed);

  //***** BUTTON PRESS SETUP ******//
	tick.attach_ms(50, tock);
  button.attachClick(buttonClick);
  button.attachLongPressStart(buttonLongPressStart);
  button.attachLongPressStop(buttonLongPressStop);
  button.attachDuringLongPress(buttonLongPress);

  Homie.disableLogging(); // before Homie.setup()
  Homie_setBrand("BUNNY")
  Homie_setFirmware("BUNNY", "3.0.0a");
  Homie.onEvent(onHomieEvent);

  Homie.setSetupFunction(setupHandler);
  Homie.setLoopFunction(loopHandler);
  Homie.setup();
}

void loop() {
  Homie.loop();
}
