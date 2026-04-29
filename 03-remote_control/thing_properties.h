#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <Arduino_NetworkConfigurator.h>
#include "configuratorAgents/agents/BLEAgent.h"
#include "configuratorAgents/agents/SerialAgent.h"
void onLEFTMOTORSPEEDChange();
void onRIGHTMOTORSPEEDChange();
void onAUTOONChange();

int DISTANCE;
int LDR_THING;
int LEFT_MOTOR_SPEED;
int RIGHT_MOTOR_SPEED;
bool AUTO_ON;

KVStore kvStore;
BLEAgentClass BLEAgent;
SerialAgentClass SerialAgent;
WiFiConnectionHandler ArduinoIoTPreferredConnection; 
NetworkConfiguratorClass NetworkConfigurator(ArduinoIoTPreferredConnection);

void initProperties(){
  NetworkConfigurator.addAgent(BLEAgent);
  NetworkConfigurator.addAgent(SerialAgent);
  NetworkConfigurator.setStorage(kvStore);
  // For changing the default reset pin uncomment and set your preferred pin. Use DISABLE_PIN for disabling the reset procedure.
  //NetworkConfigurator.setReconfigurePin(your_pin);
  ArduinoCloud.setConfigurator(NetworkConfigurator);

  ArduinoCloud.addProperty(DISTANCE, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(LDR_THING, READ, 2 * SECONDS, NULL);
  ArduinoCloud.addProperty(LEFT_MOTOR_SPEED, READWRITE, ON_CHANGE, onLEFTMOTORSPEEDChange);
  ArduinoCloud.addProperty(RIGHT_MOTOR_SPEED, READWRITE, ON_CHANGE, onRIGHTMOTORSPEEDChange);
  ArduinoCloud.addProperty(AUTO_ON, READWRITE, ON_CHANGE, onAUTOONChange);

}

