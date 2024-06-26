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
#include <OXRS_HASS.h>                // For Home Assistant self-discovery
#include <KnxTpUart.h>                // For KNX BCU

#if defined(OXRS_RACK32)
#include <OXRS_Rack32.h>              // Rack32 support
OXRS_Rack32 oxrs;
#elif defined(OXRS_BLACK)
#include <OXRS_Black.h>               // Black support
OXRS_Black oxrs;
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
#define       KNX_DEFAULT_ADDRESS   KNX_IA(1, 1, 244)
#define       KNX_SERIAL_BAUD       19200
#define       KNX_SERIAL_CONFIG     SERIAL_8E1
#define       KNX_SERIAL_RX         16
#define       KNX_SERIAL_TX         17

// Max number of supported inputs
const uint8_t MAX_INPUT_COUNT       = MCP_COUNT * MCP_PIN_COUNT;

/*-------------------------- Internal datatypes --------------------------*/
// Used to store KNX config/state
struct KNXConfig
{
  // config option to only send KNX commands if in failover mode
  bool failoverOnly;

  // address for sending on/off/up/down commands to the KNX actuator
  uint16_t commandAddress;
  // address for listening for status messages from the KNX actuator
  uint16_t stateAddress;

  // current state of the KNX actuator
  bool state;

  // set to true to trigger a GroupRead request
  bool stateRead;
};

/*--------------------------- Global Variables ------------------------*/
// Each bit corresponds to an MCP found on the IC2 bus
uint8_t g_mcps_found = 0;

// Query current value of all bi-stable inputs
bool g_queryInputs = false;

// Publish Home Assistant self-discovery config for each input
bool g_hassDiscoveryPublished[MAX_INPUT_COUNT];

// Force KNX failover flag
bool g_forceFailover = false;

// KNX config for every input
KNXConfig g_knx_config[MAX_INPUT_COUNT];

/*--------------------------- Instantiate Globals ---------------------*/
// I/O buffers
Adafruit_MCP23X17 mcp23017[MCP_COUNT];

// Input handlers
OXRS_Input oxrsInput[MCP_COUNT];

// Home Assistant self-discovery
OXRS_HASS hass(oxrs.getMQTT());

// KNX BCU on Serial2
KnxTpUart knx(&Serial2, KNX_DEFAULT_ADDRESS);

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
  JsonArray typeEnum = parent["enum"].to<JsonArray>();
  
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
        case RELEASE_EVENT:
          sprintf_P(eventType, PSTR("release"));
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
          sprintf_P(eventType, PSTR("open"));
          break;
        case HIGH_EVENT:
          sprintf_P(eventType, PSTR("closed"));
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
        case LOW_EVENT:
          sprintf_P(eventType, PSTR("alarm"));
          break;
        case HIGH_EVENT:
          sprintf_P(eventType, PSTR("normal"));
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
  #if defined(OXRS_LCD_ENABLE)
  switch (inputType)
  {
    case SECURITY:
      oxrs.getLCD()->setPinType(mcp, pin, PIN_TYPE_SECURITY);
      break;
    default:
      oxrs.getLCD()->setPinType(mcp, pin, PIN_TYPE_DEFAULT);
      break;
  }
  #endif

  // Pass this update to the input handler
  oxrsInput[mcp].setType(pin, inputType);
}

void setInputInvert(uint8_t mcp, uint8_t pin, int invert)
{
  // Configure the display
  #if defined(OXRS_LCD_ENABLE)
  oxrs.getLCD()->setPinInvert(mcp, pin, invert);
  #endif

  // Pass this update to the input handler
  oxrsInput[mcp].setInvert(pin, invert);
}

void setInputDisabled(uint8_t mcp, uint8_t pin, int disabled)
{
  // Configure the display
  #if defined(OXRS_LCD_ENABLE)
  oxrs.getLCD()->setPinDisabled(mcp, pin, disabled);
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
bool knxTelegramCheck(KnxTelegram * telegram)
{
  // Check this is a message sent to a target group 
  if (!telegram->isTargetGroup())
    return false;

  // Get the telegram address to save looking up for each loop iteration
  uint16_t targetAddress = telegram->getTargetGroupAddress();

  // Ensure we show interest where required, so an ACK can be sent 
  for (uint8_t i = 0; i < MAX_INPUT_COUNT; i++)
  {
    if (g_knx_config[i].stateAddress == targetAddress)
    {
      return true;
    }
  }

  return false;
}

void knxTelegram(KnxTelegram * telegram, bool interesting)
{
  // Ignore any telegrams we didn't identify as being interesting
  if (!interesting)
    return;

  // Only interested in write/response telegrams - i.e. a device publishing state 
  if (telegram->getCommand() != KNX_COMMAND_WRITE && telegram->getCommand() != KNX_COMMAND_ANSWER)
    return;

  // Only interested in 1-bit (bool) values
  if (telegram->getPayloadLength() != 2)
    return;

  // Get the telegram address/value to save looking up for each loop iteration
  uint16_t targetAddress = telegram->getTargetGroupAddress();
  bool value = telegram->getBool();

  // Update our internal state for any inputs with this stateAddress
  for (uint8_t i = 0; i < MAX_INPUT_COUNT; i++)
  {
    if (g_knx_config[i].stateAddress == targetAddress)
    {
      g_knx_config[i].state = value;
    }
  }
}

void initialiseKnx()
{
  // Listen for telegrams addressed to our KNX state addresses 
  knx.setTelegramCheckCallback(knxTelegramCheck);
  knx.setKnxTelegramCallback(knxTelegram);

  // Configure the second serial port on the ESP32 for the KNX BCU
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

void loopKnx()
{
  // Check for any events on the KNX bus
  knx.serialEvent();

  // Check if we have any inputs waiting for GroupRead requests (one per loop)
  for (uint8_t i = 0; i < MAX_INPUT_COUNT; i++)
  {
    if (!g_knx_config[i].stateRead)
      continue;

    if (g_knx_config[i].stateAddress != 0)
    {
      knx.groupRead(g_knx_config[i].stateAddress);
    }

    g_knx_config[i].stateRead = false;
    break;
  }
}

void publishKnxEvent(uint8_t index, uint8_t type, uint8_t state)
{
  // Get the KNX group address configured for this input (if any)...
  uint16_t commandAddress = g_knx_config[index - 1].commandAddress;
  
  // Ignore if no KNX command address configured  
  if (commandAddress == 0)
    return;

  // Determine what type of KNX telegram to send...
  switch (type)
  {
    case BUTTON:
      // Only handle single-press events, treat as TOGGLE
      if (state == 1)
      {
        knx.groupWriteBool(commandAddress, !g_knx_config[index - 1].state);
      }
      break;
    case ROTARY:
      // Send relative inc/dec dimming telegram (no internal state needed)
      knx.groupWrite4BitDim(commandAddress, state == LOW_EVENT, 5);
      break;
    case SWITCH:
      // Send boolean telegram (no internal state needed)
      knx.groupWriteBool(commandAddress, state == LOW_EVENT);
      break;
    case PRESS:
    case TOGGLE:
      // Send boolean telegram with toggled state
      knx.groupWriteBool(commandAddress, !g_knx_config[index - 1].state);
      break;  
  }
}

void createKnxValueEnum(JsonObject parent)
{
  JsonArray valueEnum = parent["enum"].to<JsonArray>();
  
  valueEnum.add("on");
  valueEnum.add("off");
  valueEnum.add("up");
  valueEnum.add("down");
}

/**
  Config handler
 */
void setConfigSchema()
{
  // Define our config schema
  JsonDocument json;

  JsonObject knxDeviceAddress = json["knxDeviceAddress"].to<JsonObject>();
  knxDeviceAddress["title"] = "KNX Device Address";
  knxDeviceAddress["description"] = "The physical address of this device on the KNX bus. Defaults to 1.1.244.";
  knxDeviceAddress["type"] = "string";
  knxDeviceAddress["pattern"] = "^\\d+\\.\\d+\\.\\d+$";

  JsonObject defaultInputType = json["defaultInputType"].to<JsonObject>();
  defaultInputType["title"] = "Default Input Type";
  defaultInputType["description"] = "Set the default input type for anything without explicit configuration below. Defaults to ‘switch’.";
  createInputTypeEnum(defaultInputType);

  JsonObject inputs = json["inputs"].to<JsonObject>();
  inputs["title"] = "Input Configuration";
  inputs["description"] = "Add configuration for each input in use on your device. The 1-based index specifies which input you wish to configure. The type defines how an input is monitored and what events are emitted. The KNX group addresses must be in standard 3-level format, e.g. 1/2/3.";
  inputs["type"] = "array";
  
  JsonObject items = inputs["items"].to<JsonObject>();
  items["type"] = "object";

  JsonObject properties = items["properties"].to<JsonObject>();

  JsonObject index = properties["index"].to<JsonObject>();
  index["title"] = "Index";
  index["type"] = "integer";
  index["minimum"] = 1;
  index["maximum"] = getMaxIndex();

  JsonObject type = properties["type"].to<JsonObject>();
  type["title"] = "Type";
  createInputTypeEnum(type);

  JsonObject invert = properties["invert"].to<JsonObject>();
  invert["title"] = "Invert";
  invert["type"] = "boolean";

  JsonObject disabled = properties["disabled"].to<JsonObject>();
  disabled["title"] = "Disabled";
  disabled["type"] = "boolean";

  JsonObject knxCommandAddress = properties["knxCommandAddress"].to<JsonObject>();
  knxCommandAddress["title"] = "KNX Command Address";
  knxCommandAddress["type"] = "string";
  knxCommandAddress["pattern"] = "^\\d+\\/\\d+\\/\\d+$";

  JsonObject knxStateAddress = properties["knxStateAddress"].to<JsonObject>();
  knxStateAddress["title"] = "KNX State Address";
  knxStateAddress["type"] = "string";
  knxStateAddress["pattern"] = "^\\d+\\/\\d+\\/\\d+$";

  JsonObject knxFailoverOnly = properties["knxFailoverOnly"].to<JsonObject>();
  knxFailoverOnly["title"] = "KNX Failover Only";
  knxFailoverOnly["type"] = "boolean";

  JsonArray required = items["required"].to<JsonArray>();
  required.add("index");

  // Add any Home Assistant config
  hass.setConfigSchema(json);

  // Pass our config schema down to the hardware library
  oxrs.setConfigSchema(json.as<JsonVariant>());
}

uint16_t parseDeviceAddress(const char * address)
{
  const char * delimiter = ".";

  char buffer[strlen(address) + 1];
  strcpy(buffer, address);

  char * token = strtok(buffer, delimiter);
  int area = atoi(token);
  token = strtok(NULL, delimiter);
  int line = atoi(token);
  token = strtok(NULL, delimiter);
  int member = atoi(token);

  return KNX_IA(area, line, member);
}

uint16_t parseGroupAddress(const char * address)
{
  const char * delimiter = "/";

  char buffer[strlen(address) + 1];
  strcpy(buffer, address);

  char * token = strtok(buffer, delimiter);
  int main = atoi(token);
  token = strtok(NULL, delimiter);
  int mid = atoi(token);
  token = strtok(NULL, delimiter);
  int sub = atoi(token);

  return KNX_GA(main, mid, sub);
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
      g_hassDiscoveryPublished[index - 1] = false;
    }
  }
  
  if (json.containsKey("invert"))
  {
    setInputInvert(mcp, pin, json["invert"].as<bool>());
    g_hassDiscoveryPublished[index - 1] = false;
  }

  if (json.containsKey("disabled"))
  {
    setInputDisabled(mcp, pin, json["disabled"].as<bool>());
    g_hassDiscoveryPublished[index - 1] = false;
  }

  if (json.containsKey("knxCommandAddress"))
  {
    g_knx_config[index - 1].commandAddress = parseGroupAddress(json["knxCommandAddress"]);
  }

  if (json.containsKey("knxStateAddress"))
  {
    g_knx_config[index - 1].stateAddress = parseGroupAddress(json["knxStateAddress"]);
    g_knx_config[index - 1].stateRead = true;
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
    knx.setIndividualAddress(parseDeviceAddress(json["knxDeviceAddress"]));
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

  // Handle any Home Assistant config
  hass.parseConfig(json);
}

/**
  Command handler
 */
void setCommandSchema()
{
  // Define our command schema
  JsonDocument json;

  JsonObject queryInputs = json["queryInputs"].to<JsonObject>();
  queryInputs["title"] = "Query Inputs";
  queryInputs["description"] = "Query and publish the state of all bi-stable inputs.";
  queryInputs["type"] = "boolean";

  JsonObject forceFailover = json["forceFailover"].to<JsonObject>();
  forceFailover["title"] = "Force KNX";
  forceFailover["description"] = "By-pass publishing input events to MQTT and always publish to KNX, regardless of IP/MQTT connection state.";
  forceFailover["type"] = "boolean";

  JsonObject knxCommands = json["knxCommands"].to<JsonObject>();
  knxCommands["title"] = "KNX Commands";
  knxCommands["description"] = "Send one or more telegrams directly onto the KNX bus.";
  knxCommands["type"] = "array";
  
  JsonObject knxCommandItems = knxCommands["items"].to<JsonObject>();
  knxCommandItems["type"] = "object";

  JsonObject knxCommandProperties = knxCommandItems["properties"].to<JsonObject>();

  JsonObject knxGroupAddress = knxCommandProperties["knxGroupAddress"].to<JsonObject>();
  knxGroupAddress["title"] = "KNX Group Address";
  knxGroupAddress["type"] = "string";
  knxGroupAddress["pattern"] = "^\\d+\\/\\d+\\/\\d+$";

  JsonObject knxValue = knxCommandProperties["knxValue"].to<JsonObject>();
  knxValue["title"] = "KNX Value";
  createKnxValueEnum(knxValue);

  // Pass our command schema down to the hardware library
  oxrs.setCommandSchema(json.as<JsonVariant>());
}

void jsonCommand(JsonVariant json)
{
  if (json.containsKey("queryInputs"))
  {
    g_queryInputs = json["queryInputs"].as<bool>();
  }

  if (json.containsKey("forceFailover"))
  {
    g_forceFailover = json["forceFailover"].as<bool>();
  }

  if (json.containsKey("knxCommands"))
  {
    for (JsonVariant command : json["knxCommands"].as<JsonArray>())
    {
      uint16_t address = parseGroupAddress(command["knxGroupAddress"]);

      if (strcmp(command["knxValue"], "on") == 0)
      {
        knx.groupWriteBool(address, true);
      }
      else if (strcmp(command["knxValue"], "off") == 0)
      {
        knx.groupWriteBool(address, false);
      }
      else if (strcmp(command["knxValue"], "up") == 0)
      {
        knx.groupWriteBool(address, false);
      }
      else if (strcmp(command["knxValue"], "down") == 0)
      {
        knx.groupWriteBool(address, true);
      }
    }
  }
}

void publishEvent(uint8_t index, uint8_t type, uint8_t state)
{
  // Calculate the port and channel for this index (all 1-based)
  uint8_t port = ((index - 1) / 4) + 1;
  uint8_t channel = index - ((port - 1) * 4);
  
  char inputType[9];
  getInputType(inputType, type);
  char eventType[8];
  getEventType(eventType, type, state);

  JsonDocument json;
  json["port"] = port;
  json["channel"] = channel;
  json["index"] = index;
  json["type"] = inputType;
  json["event"] = eventType;

  // Always publish this event to MQTT, unless in forced failover
  bool failover = g_forceFailover;
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

void publishHassDiscovery(uint8_t mcp)
{
  char component[16];
  sprintf_P(component, PSTR("binary_sensor"));

  char inputId[16];
  char inputName[16];

  char statusTopic[64];
  char valueTemplate[128];

  // Read security sensor values in quads (a full port)
  uint8_t securityCount = 0;

  for (uint8_t pin = 0; pin < MCP_PIN_COUNT; pin++)
  {
    // Determine the input type
    uint8_t inputType = oxrsInput[mcp].getType(pin);

    // Only generate config for the last security input
    if (inputType == SECURITY)
    {
      if (++securityCount < 4) 
        continue;
      
      securityCount = 0;
    }

    // Calculate the 1-based input index
    uint8_t input = (MCP_PIN_COUNT * mcp) + pin + 1;

    // Ignore if we have already published the discovery config for this input
    if (g_hassDiscoveryPublished[input - 1])
      continue;

    // Only interested in CONTACT, SECURITY, SWITCH inputs
    if (inputType != CONTACT && inputType != SECURITY && inputType != SWITCH)
      continue;

    // JSON config payload (empty if the input is disabled, to clear any existing config)
    JsonDocument json;

    sprintf_P(inputId, PSTR("input_%d"), input);

    // Check if this input is disabled
    if (!oxrsInput[mcp].getDisabled(pin))
    {
      hass.getDiscoveryJson(json, inputId);

      sprintf_P(inputName, PSTR("Input %d"), input);
      switch (inputType)
      {
        case CONTACT:
          sprintf_P(valueTemplate, PSTR("{%% if value_json.index == %d %%}{%% if value_json.event == 'open' %%}ON{%% else %%}OFF{%% endif %%}{%% endif %%}"), input);
          break;
        case SECURITY:
          sprintf_P(valueTemplate, PSTR("{%% if value_json.index == %d %%}{%% if value_json.event == 'alarm' %%}ON{%% else %%}OFF{%% endif %%}{%% endif %%}"), input);
          break;
        case SWITCH:
          sprintf_P(valueTemplate, PSTR("{%% if value_json.index == %d %%}{%% if value_json.event == 'on' %%}ON{%% else %%}OFF{%% endif %%}{%% endif %%}"), input);
          break;
      }

      json["name"] = inputName;
      json["stat_t"] = oxrs.getMQTT()->getStatusTopic(statusTopic);
      json["val_tpl"] = valueTemplate;
    }

    // Publish retained and stop trying once successful 
    g_hassDiscoveryPublished[input - 1] = hass.publishDiscoveryJson(json, component, inputId);
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
  #if defined(OXRS_LCD_ENABLE)
  oxrs.getLCD()->drawPorts(PORT_LAYOUT_INPUT_AUTO, g_mcps_found);
  #endif

  // Set up config/command schemas (for self-discovery and adoption)
  setConfigSchema();
  setCommandSchema();
  
  // Speed up I2C clock for faster scan rate (after bus scan)
  Wire.setClock(I2C_CLOCK_SPEED);

  // Set up KNX callbacks and serial comms to BCU
  initialiseKnx();
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
    #if defined(OXRS_LCD_ENABLE)
    oxrs.getLCD()->process(mcp, io_value);
    #endif

    // Check for any input events
    oxrsInput[mcp].process(mcp, io_value);

    // Check if we are querying the current values
    if (g_queryInputs)
    {
      oxrsInput[mcp].queryAll(mcp);
    }

    // Check if we need to publish any Home Assistant discovery payloads
    if (hass.isDiscoveryEnabled())
    {
      publishHassDiscovery(mcp);
    }
  }

  // Ensure we don't keep querying
  g_queryInputs = false;

  // Check for KNX events
  loopKnx();
}
