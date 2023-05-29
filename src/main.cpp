/**
  KNX state monitor firmware for the Open eXtensible Rack System

  Documentation:  
    https://oxrs.io/docs/firmware/state-monitor-knx.html

  Supported hardware:
    https://www.superhouse.tv/product/i2c-rj45-light-switch-breakout/

  GitHub repository:
    https://github.com/sumnerboy12/OXRS-BJ-StateMonitor-KNX-FW

  Copyright 2023 Ben Jones <ben.jones12@gmail.com>
*/

/*--------------------------- Libraries -------------------------------*/
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>        // For MCP23017 I/O buffers
#include <OXRS_Input.h>               // For input handling
#include <KnxTpUart.h>                // For KNX BCU

#if defined(OXRS_RACK32)
#include <OXRS_Rack32.h>              // Rack32 support
OXRS_Rack32 oxrs;
#endif

/*--------------------------- Constants -------------------------------*/
// Serial
#define       SERIAL_BAUD_RATE      115200

// Can have up to 8x MCP23017s on a single I2C bus
const byte    MCP_I2C_ADDRESS[]     = { 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 };
const uint8_t MCP_COUNT             = sizeof(MCP_I2C_ADDRESS);

// Each MCP23017 has 16 I/O pins
#define       MCP_PIN_COUNT         16

// Set false for breakout boards with external pull-ups
#define       MCP_INTERNAL_PULLUPS  true

// Speed up the I2C bus to get faster event handling
#define       I2C_CLOCK_SPEED       400000L

// Internal constant used when input type parsing fails
#define       INVALID_INPUT_TYPE    99

// KNX BCU on Serial2
#define       KNX_SERIAL_BAUD       19200
#define       KNX_SERIAL_CONFIG     SERIAL_8E1
#define       KNX_SERIAL_RX         16
#define       KNX_SERIAL_TX         17

/*-------------------------- Internal datatypes --------------------------*/
// Used to store KNX config/state
struct KNXConfig
{
  bool failoverOnly;

  uint16_t knxCommandAddress;
  uint16_t knxStateAddress;

  bool state;
};

/*--------------------------- Global Variables ------------------------*/
// Each bit corresponds to an MCP found on the IC2 bus
uint8_t g_mcps_found = 0;

// Force KNX flag
bool g_force_knx = false;

// KNX config for each possible input
KNXConfig g_knx_config[MCP_COUNT * MCP_PIN_COUNT];

/*--------------------------- Instantiate Globals ---------------------*/
// I/O buffers
Adafruit_MCP23X17 mcp23017[MCP_COUNT];

// Input handlers
OXRS_Input oxrsInput[MCP_COUNT];

// KNX BCU on Serial2
KnxTpUart knx(&Serial2, KNX_IA(1, 1, 244));

/*--------------------------- Program ---------------------------------*/
uint8_t getMaxIndex()
{
  // Count how many MCPs were found
  uint8_t mcpCount = 0;
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    if (bitRead(g_mcps_found, mcp) != 0) { mcpCount++; }
  }

  // Remember our indexes are 1-based
  return mcpCount * MCP_PIN_COUNT;  
}

void createInputTypeEnum(JsonObject parent)
{
  JsonArray typeEnum = parent.createNestedArray("enum");
  
  typeEnum.add("button");
  typeEnum.add("contact");
  typeEnum.add("press");
  typeEnum.add("rotary");
  typeEnum.add("security");
  typeEnum.add("switch");
  typeEnum.add("toggle");
}

uint8_t parseInputType(const char * inputType)
{
  if (strcmp(inputType, "button")   == 0) { return BUTTON; }
  if (strcmp(inputType, "contact")  == 0) { return CONTACT; }
  if (strcmp(inputType, "press")    == 0) { return PRESS; }
  if (strcmp(inputType, "rotary")   == 0) { return ROTARY; }
  if (strcmp(inputType, "security") == 0) { return SECURITY; }
  if (strcmp(inputType, "switch")   == 0) { return SWITCH; }
  if (strcmp(inputType, "toggle")   == 0) { return TOGGLE; }

  oxrs.println(F("[knx] invalid input type"));
  return INVALID_INPUT_TYPE;
}

void getInputType(char inputType[], uint8_t type)
{
  // Determine what type of input we have
  sprintf_P(inputType, PSTR("error"));
  switch (type)
  {
    case BUTTON:
      sprintf_P(inputType, PSTR("button"));
      break;
    case CONTACT:
      sprintf_P(inputType, PSTR("contact"));
      break;
    case PRESS:
      sprintf_P(inputType, PSTR("press"));
      break;
    case ROTARY:
      sprintf_P(inputType, PSTR("rotary"));
      break;
    case SECURITY:
      sprintf_P(inputType, PSTR("security"));
      break;
    case SWITCH:
      sprintf_P(inputType, PSTR("switch"));
      break;
    case TOGGLE:
      sprintf_P(inputType, PSTR("toggle"));
      break;
  }
}

void getEventType(char eventType[], uint8_t type, uint8_t state)
{
  // Determine what event we need to publish
  sprintf_P(eventType, PSTR("error"));
  switch (type)
  {
    case BUTTON:
      switch (state)
      {
        case HOLD_EVENT:
          sprintf_P(eventType, PSTR("hold"));
          break;
        case 1:
          sprintf_P(eventType, PSTR("single"));
          break;
        case 2:
          sprintf_P(eventType, PSTR("double"));
          break;
        case 3:
          sprintf_P(eventType, PSTR("triple"));
          break;
        case 4:
          sprintf_P(eventType, PSTR("quad"));
          break;
        case 5:
          sprintf_P(eventType, PSTR("penta"));
          break;
      }
      break;
    case CONTACT:
      switch (state)
      {
        case LOW_EVENT:
          sprintf_P(eventType, PSTR("closed"));
          break;
        case HIGH_EVENT:
          sprintf_P(eventType, PSTR("open"));
          break;
      }
      break;
    case PRESS:
      sprintf_P(eventType, PSTR("press"));
      break;
    case ROTARY:
      switch (state)
      {
        case LOW_EVENT:
          sprintf_P(eventType, PSTR("up"));
          break;
        case HIGH_EVENT:
          sprintf_P(eventType, PSTR("down"));
          break;
      }
      break;
    case SECURITY:
      switch (state)
      {
        case HIGH_EVENT:
          sprintf_P(eventType, PSTR("normal"));
          break;
        case LOW_EVENT:
          sprintf_P(eventType, PSTR("alarm"));
          break;
        case TAMPER_EVENT:
          sprintf_P(eventType, PSTR("tamper"));
          break;
        case SHORT_EVENT:
          sprintf_P(eventType, PSTR("short"));
          break;
        case FAULT_EVENT:
          sprintf_P(eventType, PSTR("fault"));
          break;
      }
      break;
    case SWITCH:
      switch (state)
      {
        case LOW_EVENT:
          sprintf_P(eventType, PSTR("on"));
          break;
        case HIGH_EVENT:
          sprintf_P(eventType, PSTR("off"));
          break;
      }
      break;
    case TOGGLE:
      sprintf_P(eventType, PSTR("toggle"));
      break;
  }
}

void setInputType(uint8_t mcp, uint8_t pin, uint8_t inputType)
{
  // Configure the display (type constant from LCD library)
  #if defined(OXRS_RACK32)
  switch (inputType)
  {
    case SECURITY:
      oxrs.setDisplayPinType(mcp, pin, PIN_TYPE_SECURITY);
      break;
    default:
      oxrs.setDisplayPinType(mcp, pin, PIN_TYPE_DEFAULT);
      break;
  }
  #endif

  // Pass this update to the input handler
  oxrsInput[mcp].setType(pin, inputType);
}

void setInputInvert(uint8_t mcp, uint8_t pin, int invert)
{
  // Configure the display
  #if defined(OXRS_RACK32)
  oxrs.setDisplayPinInvert(mcp, pin, invert);
  #endif

  // Pass this update to the input handler
  oxrsInput[mcp].setInvert(pin, invert);
}

void setInputDisabled(uint8_t mcp, uint8_t pin, int disabled)
{
  // Configure the display
  #if defined(OXRS_RACK32)
  oxrs.setDisplayPinDisabled(mcp, pin, disabled);
  #endif

  // Pass this update to the input handler
  oxrsInput[mcp].setDisabled(pin, disabled);
}

void setDefaultInputType(uint8_t inputType)
{
  // Set all pins on all MCPs to this default input type
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    if (bitRead(g_mcps_found, mcp) == 0)
      continue;

    for (uint8_t pin = 0; pin < MCP_PIN_COUNT; pin++)
    {
      setInputType(mcp, pin, inputType);
    }
  }
}

/**
  KNX
*/
void knxTelegram(KnxTelegram * telegram, bool interesting)
{
  for (uint8_t i = 0; i < sizeof(g_knx_config); i++)
  {
    if (g_knx_config[i].knxStateAddress == telegram->getTargetGroupAddress())
    {
      g_knx_config[i].state = telegram->getBool();
    }
  }
}

void initialiseKnxSerial()
{
  // Listen for telegrams addressed to our KNX state addresses 
  knx.setKnxTelegramCallback(knxTelegram);

  oxrs.println(F("[knx] setting up Serial2 for KNX BCU..."));
  oxrs.print(F(" - baud:   "));
  oxrs.println(KNX_SERIAL_BAUD);
  oxrs.print(F(" - config: "));
  oxrs.println(KNX_SERIAL_CONFIG);
  oxrs.print(F(" - rx pin: "));
  oxrs.println(KNX_SERIAL_RX);
  oxrs.print(F(" - tx pin: "));
  oxrs.println(KNX_SERIAL_TX);

  Serial2.begin(KNX_SERIAL_BAUD, KNX_SERIAL_CONFIG, KNX_SERIAL_RX, KNX_SERIAL_TX);  
}

void publishKnxEvent(uint8_t index, uint8_t type, uint8_t state)
{
  // Determine the KNX group address to use for this index (if any)...
  uint16_t knxCommandAddress = g_knx_config[index - 1].knxCommandAddress;
  
  oxrs.print(F("[knx] knx command address "));

  if (knxCommandAddress == 0)
  {
    oxrs.println(F("not configured, ignoring"));
    return;
  }

  oxrs.println(knxCommandAddress);

  // Determine what type of KNX telegram to send...
  switch (type)
  {
    case BUTTON:
      // Ignore HOLD events and treat all multi-press events as a TOGGLE
      if (state != HOLD_EVENT)
      {
        // Toggle internal state and send boolean telegram with new state
        knx.groupWriteBool(knxCommandAddress, !g_knx_config[index - 1].state);
      }
      break;
    case ROTARY:
      // Send relative inc/dec dimming telegram (no internal state needed)
      knx.groupWrite4BitDim(knxCommandAddress, state == LOW_EVENT, 5);
      break;
    case SWITCH:
      // Send boolean telegram (no internal state needed)
      knx.groupWriteBool(knxCommandAddress, state == LOW_EVENT);
      break;
    case PRESS:
    case TOGGLE:
      // Toggle internal state and send boolean telegram with new state
      knx.groupWriteBool(knxCommandAddress, !g_knx_config[index - 1].state);
      break;  
  }
}

/**
  Config handler
 */
void setConfigSchema()
{
  // Define our config schema
  StaticJsonDocument<1024> json;

  JsonObject knxDeviceAddress = json.createNestedObject("knxDeviceAddress");
  knxDeviceAddress["title"] = "KNX Device Address";
  knxDeviceAddress["description"] = "The physical address of this device on the KNX bus. Defaults to 1.1.244.";
  knxDeviceAddress["type"] = "string";
  knxDeviceAddress["pattern"] = "^\\d+\\.\\d+\\.\\d+$";

  JsonObject defaultInputType = json.createNestedObject("defaultInputType");
  defaultInputType["title"] = "Default Input Type";
  defaultInputType["description"] = "Set the default input type for anything without explicit configuration below. Defaults to ‘switch’.";
  createInputTypeEnum(defaultInputType);

  JsonObject inputs = json.createNestedObject("inputs");
  inputs["title"] = "Input Configuration";
  inputs["description"] = "Add configuration for each input in use on your device. The 1-based index specifies which input you wish to configure. The type defines how an input is monitored and what events are emitted. The KNX group addresses must be in standard 3-level format, e.g. 1/2/3.";
  inputs["type"] = "array";
  
  JsonObject items = inputs.createNestedObject("items");
  items["type"] = "object";

  JsonObject properties = items.createNestedObject("properties");

  JsonObject index = properties.createNestedObject("index");
  index["title"] = "Index";
  index["type"] = "integer";
  index["minimum"] = 1;
  index["maximum"] = getMaxIndex();

  JsonObject type = properties.createNestedObject("type");
  type["title"] = "Type";
  createInputTypeEnum(type);

  JsonObject invert = properties.createNestedObject("invert");
  invert["title"] = "Invert";
  invert["type"] = "boolean";

  JsonObject disabled = properties.createNestedObject("disabled");
  disabled["title"] = "Disabled";
  disabled["type"] = "boolean";

  JsonObject knxCommandAddress = properties.createNestedObject("knxCommandAddress");
  knxCommandAddress["title"] = "KNX Command Address";
  knxCommandAddress["type"] = "string";
  knxCommandAddress["pattern"] = "^\\d+\\/\\d+\\/\\d+$";

  JsonObject knxStateAddress = properties.createNestedObject("knxStateAddress");
  knxStateAddress["title"] = "KNX State Address";
  knxStateAddress["type"] = "string";
  knxStateAddress["pattern"] = "^\\d+\\/\\d+\\/\\d+$";

  JsonObject knxFailoverOnly = properties.createNestedObject("knxFailoverOnly");
  knxFailoverOnly["title"] = "KNX Failover Only";
  knxFailoverOnly["type"] = "boolean";

  JsonArray required = items.createNestedArray("required");
  required.add("index");

  // Pass our config schema down to the hardware library
  oxrs.setConfigSchema(json.as<JsonVariant>());
}

uint8_t getIndex(JsonVariant json)
{
  if (!json.containsKey("index"))
  {
    oxrs.println(F("[knx] missing index"));
    return 0;
  }
  
  uint8_t index = json["index"].as<uint8_t>();

  // Check the index is valid for this device
  if (index <= 0 || index > getMaxIndex())
  {
    oxrs.println(F("[knx] invalid index"));
    return 0;
  }

  return index;
}

void jsonInputConfig(JsonVariant json)
{
  uint8_t index = getIndex(json);
  if (index == 0) return;

  // Work out the MCP and pin we are configuring
  int mcp = (index - 1) / MCP_PIN_COUNT;
  int pin = (index - 1) % MCP_PIN_COUNT;

  if (json.containsKey("type"))
  {
    uint8_t inputType = parseInputType(json["type"]);    

    if (inputType != INVALID_INPUT_TYPE)
    {
      setInputType(mcp, pin, inputType);
    }
  }
  
  if (json.containsKey("invert"))
  {
    setInputInvert(mcp, pin, json["invert"].as<bool>());
  }

  if (json.containsKey("disabled"))
  {
    setInputDisabled(mcp, pin, json["disabled"].as<bool>());
  }

  if (json.containsKey("knxCommandAddress"))
  {
    g_knx_config[index - 1].knxCommandAddress = knx.getGroupAddress(json["knxCommandAddress"]);
  }

  if (json.containsKey("knxStateAddress"))
  {
    g_knx_config[index - 1].knxStateAddress = knx.getGroupAddress(json["knxStateAddress"]);
  }

  if (json.containsKey("knxFailoverOnly"))
  {
    g_knx_config[index - 1].failoverOnly = json["knxFailoverOnly"].as<bool>();
  }
}

void jsonConfig(JsonVariant json)
{
  if (json.containsKey("knxDeviceAddress"))
  {
    const char * knxDeviceAddress = json["knxDeviceAddress"];
    const char * delimiter = ".";

    char buffer[strlen(knxDeviceAddress) + 1];
    strcpy(buffer, knxDeviceAddress);

    char * token = strtok(buffer, delimiter);
    int area = atoi(token);
    token = strtok(NULL, delimiter);
    int line = atoi(token);
    token = strtok(NULL, delimiter);
    int member = atoi(token);

    knx.setIndividualAddress(KNX_IA(area, line, member));
  }

  if (json.containsKey("defaultInputType"))
  {
    uint8_t inputType = parseInputType(json["defaultInputType"]);

    if (inputType != INVALID_INPUT_TYPE)
    {
      setDefaultInputType(inputType);
    }
  }

  if (json.containsKey("inputs"))
  {
    for (JsonVariant input : json["inputs"].as<JsonArray>())
    {
      jsonInputConfig(input);    
    }
  }
}

/**
  Command handler
 */
void setCommandSchema()
{
  // Define our command schema
  StaticJsonDocument<1024> json;

  JsonObject forceKnx = json.createNestedObject("forceKnx");
  forceKnx["title"] = "Force KNX";
  forceKnx["description"] = "By-pass publishing input events to MQTT and always publish to KNX, regardless of IP/MQTT connection state.";
  forceKnx["type"] = "boolean";

  // Pass our command schema down to the hardware library
  oxrs.setCommandSchema(json.as<JsonVariant>());
}

void jsonCommand(JsonVariant json)
{
  if (json.containsKey("forceKnx"))
  {
    g_force_knx = json["forceKnx"].as<bool>();
  }
}

void publishEvent(uint8_t index, uint8_t type, uint8_t state)
{
  // Calculate the port and channel for this index (all 1-based)
  uint8_t port = ((index - 1) / 4) + 1;
  uint8_t channel = index - ((port - 1) * 4);
  
  char inputType[9];
  getInputType(inputType, type);
  char eventType[7];
  getEventType(eventType, type, state);

  StaticJsonDocument<128> json;
  json["port"] = port;
  json["channel"] = channel;
  json["index"] = index;
  json["type"] = inputType;
  json["event"] = eventType;

  // Always publish this event to MQTT, unless in forced failover
  bool failover = g_force_knx;
  if (!failover)
  {
    failover = !oxrs.publishStatus(json.as<JsonVariant>());
  }

  // Always publish this event to KNX, unless not in failover and failover-only enabled
  if (failover || !g_knx_config[index - 1].failoverOnly)
  {
    publishKnxEvent(index, type, state);
  }
}

/**
  Event handlers
*/
void inputEvent(uint8_t id, uint8_t input, uint8_t type, uint8_t state)
{
  // Determine the index for this input event (1-based)
  uint8_t mcp = id;
  uint8_t index = (MCP_PIN_COUNT * mcp) + input + 1;

  // Publish the event
  publishEvent(index, type, state);
}

/**
  I2C
*/
void scanI2CBus()
{
  oxrs.println(F("[knx] scanning for I/O buffers..."));

  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    oxrs.print(F(" - 0x"));
    oxrs.print(MCP_I2C_ADDRESS[mcp], HEX);
    oxrs.print(F("..."));

    // Check if there is anything responding on this address
    Wire.beginTransmission(MCP_I2C_ADDRESS[mcp]);
    if (Wire.endTransmission() == 0)
    {
      bitWrite(g_mcps_found, mcp, 1);
      
      // If an MCP23017 was found then initialise and configure the inputs
      mcp23017[mcp].begin_I2C(MCP_I2C_ADDRESS[mcp]);
      for (uint8_t pin = 0; pin < MCP_PIN_COUNT; pin++)
      {
        mcp23017[mcp].pinMode(pin, MCP_INTERNAL_PULLUPS ? INPUT_PULLUP : INPUT);
      }

      // Initialise input handlers (default to SWITCH)
      oxrsInput[mcp].begin(inputEvent, SWITCH);

      oxrs.print(F("MCP23017"));
      if (MCP_INTERNAL_PULLUPS) { oxrs.print(F(" (internal pullups)")); }
      oxrs.println();
    }
    else
    {
      oxrs.println(F("empty"));
    }
  }
}

/**
  Setup
*/
void setup()
{
  // Start serial and let settle
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  Serial.println(F("[knx] starting up..."));

  // Start the I2C bus
  Wire.begin(I2C_SDA, I2C_SCL);

  // Scan the I2C bus and set up I/O buffers
  scanI2CBus();

  // Start hardware
  oxrs.begin(jsonConfig, jsonCommand);

  // Set up port display
  #if defined(OXRS_RACK32)
  oxrs.setDisplayPortLayout(g_mcps_found, PORT_LAYOUT_INPUT_AUTO);
  #endif

  // Set up config/command schemas (for self-discovery and adoption)
  setConfigSchema();
  setCommandSchema();
  
  // Speed up I2C clock for faster scan rate (after bus scan)
  Wire.setClock(I2C_CLOCK_SPEED);

  // Set up the KNX serial port
  initialiseKnxSerial();
}

/**
  Main processing loop
*/
void loop()
{
  // Let hardware handle any events etc
  oxrs.loop();

  // Iterate through each of the MCP23017s
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    if (bitRead(g_mcps_found, mcp) == 0)
      continue;

    // Read the values for all 16 pins on this MCP
    uint16_t io_value = mcp23017[mcp].readGPIOAB();

    // Show port animations
    #if defined(OXRS_RACK32)
    oxrs.updateDisplayPorts(mcp, io_value);
    #endif

    // Check for any input events
    oxrsInput[mcp].process(mcp, io_value);
  }

  // Check for KNX events
  knx.serialEvent();
}
