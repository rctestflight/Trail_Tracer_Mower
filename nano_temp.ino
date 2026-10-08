#include <SPI.h>
#include <mcp_can.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// MCP2515: VCC=5V, GND=GND, CS=D10, SO=D12, SI=D11, SCK=D13, INT=D2.
const uint8_t NODE_ID = 5; // Match ODrive axis node ID, 0..62.
const uint8_t BITRATE = CAN_250KBPS;
const uint8_t MCP_CLOCK = MCP_8MHZ; // Change to MCP_16MHZ if crystal is 16 MHz.
const float MAX_RPM = 300;
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;
MCP_CAN canBus(10);
bool seen = false, armed = false, pending = false;
uint8_t axisState = 0;
uint32_t axisError = 0, heartbeatMs = 0, armMs = 0, commandMs = 0;
float targetRPM = 0, measuredRPM = 0;
char line[40];
uint8_t used = 0;
bool overflow = false;
uint32_t rxFrames = 0, txFailures = 0, lastRxId = 0;
uint32_t scanStartedMs = 0;
bool scanning = false;
uint8_t lastTxResult = CAN_OK;

bool sendCAN(uint8_t cmd, uint8_t len, uint8_t *data) {
  lastTxResult = canBus.sendMsgBuf((uint16_t(NODE_ID) << 5) | cmd, 0, len, data);
  if (lastTxResult != CAN_OK) ++txFailures;
  return lastTxResult == CAN_OK;
}
bool state(uint32_t value) {
  uint8_t data[4];
  memcpy(data, &value, 4); // AVR and CANSimple are little endian.
  return sendCAN(0x07, 4, data);
}
bool velocity(float rpm) {
  uint8_t data[8] = {0}; // Zero torque feedforward.
  float turns = rpm / 60.0f;
  memcpy(data, &turns, 4);
  return sendCAN(0x0D, 8, data);
}
bool fresh() { return seen && millis() - heartbeatMs < HEARTBEAT_TIMEOUT_MS; }
void stopMotor() {
  armed = pending = false;
  targetRPM = 0;
  velocity(0);
  state(1); // IDLE; best effort. Enable drive watchdog for disconnected CAN.
}
void receiveCAN() {
  // Poll the controller directly so a missing INT connection cannot hide frames.
  for (uint8_t n = 0; n < 8 && canBus.checkReceive() == CAN_MSGAVAIL; ++n) {
    unsigned long id;
    uint8_t len, data[8];
    if (canBus.readMsgBuf(&id, &len, data) != CAN_OK) break;
    ++rxFrames;
    lastRxId = id;
    if (scanning && !(id & 0xC0000000UL) && (id & 31) == 1 && len >= 5) {
      uint32_t error;
      memcpy(&error, data, 4);
      Serial.print(F("RX heartbeat node=")); Serial.print(id >> 5);
      Serial.print(F(" state=")); Serial.print(data[4]);
      Serial.print(F(" error=0x")); Serial.println(error, HEX);
    }
    if ((id & 0xC0000000UL) || (id >> 5) != NODE_ID) continue;
    if ((id & 31) == 1 && len >= 5) {
      memcpy(&axisError, data, 4);
      axisState = data[4];
      heartbeatMs = millis();
      seen = true;
    } else if ((id & 31) == 9 && len == 8) {
      memcpy(&measuredRPM, data + 4, 4);
      measuredRPM *= 60;
    }
  }
}
void processLine() {
  if (!strcmp(line, "scan")) {
    scanning = true; scanStartedMs = millis();
    Serial.println(F("Listening for all node heartbeats for 3 seconds"));
  } else if (!strcmp(line, "probe")) {
    // Heartbeat RTR request: no motion command. Also supplies CAN traffic for autobaud.
    uint8_t data[8] = {0};
    lastTxResult = canBus.sendMsgBuf(((uint32_t(NODE_ID) << 5) | 1) | 0x40000000UL,
                                    8, data);
    if (lastTxResult != CAN_OK) ++txFailures;
    Serial.print(F("Heartbeat request TX result=")); Serial.println(lastTxResult);
  } else if (!strcmp(line, "clear")) {
    if (armed || pending) { Serial.println(F("Send idle before clear")); return; }
    uint8_t data[8] = {0};
    // Match the working example's zero-length Clear_Errors command.
    Serial.println(sendCAN(0x18, 0, data) ? F("Clear errors sent; check status") : F("Clear TX failed"));
  } else if (!strcmp(line, "idle")) {
    stopMotor(); Serial.println(F("IDLE requested"));
  } else if (!strcmp(line, "status")) {
    Serial.print(F("heartbeat=")); Serial.print(fresh());
    Serial.print(F(" state=")); Serial.print(axisState);
    Serial.print(F(" error=0x")); Serial.print(axisError, HEX);
    Serial.print(F(" armed=")); Serial.print(armed);
    Serial.print(F(" target RPM=")); Serial.print(targetRPM);
    Serial.print(F(" last measured RPM=")); Serial.println(measuredRPM);
    Serial.print(F("node=")); Serial.print(NODE_ID);
    Serial.print(F(" crystal MHz=")); Serial.println(MCP_CLOCK == MCP_8MHZ ? 8 : 16);
    Serial.print(F("rx frames=")); Serial.print(rxFrames);
    Serial.print(F(" last id=0x")); Serial.print(lastRxId, HEX);
    Serial.print(F(" tx failures=")); Serial.print(txFailures);
    Serial.print(F(" last tx result=")); Serial.println(lastTxResult);
    Serial.print(F("MCP EFLG=0x")); Serial.print(canBus.getError(), HEX);
    Serial.print(F(" REC=")); Serial.print(canBus.errorCountRX());
    Serial.print(F(" TEC=")); Serial.println(canBus.errorCountTX());
    if (seen) { Serial.print(F("Heartbeat age ms=")); Serial.println(millis() - heartbeatMs); }
  } else if (!strcmp(line, "arm")) {
    if (armed || pending || !fresh() || axisError || axisState != 1) {
      Serial.println(F("Need fresh error-free IDLE heartbeat; already armed/pending?"));
      return;
    }
    uint32_t mode = 2, input = 1; // Velocity control, passthrough.
    uint8_t data[8];
    memcpy(data, &mode, 4); memcpy(data + 4, &input, 4);
    targetRPM = 0;
    if (!sendCAN(0x0B, 8, data) || !velocity(0) || !state(8)) {
      stopMotor(); Serial.println(F("CAN transmit failed")); return;
    }
    pending = true; armMs = millis();
    Serial.println(F("Waiting for closed loop"));
  } else if (!strncmp(line, "v ", 2)) {
    char *end;
    float rpm = strtod(line + 2, &end);
    bool parsed = end != line + 2;
    while (*end == ' ') ++end;
    if (!parsed || *end || !isfinite(rpm)) Serial.println(F("Use v <RPM>"));
    else if (!armed) Serial.println(F("Arm first; wait for Armed"));
    else {
      targetRPM = constrain(rpm, -MAX_RPM, MAX_RPM);
      Serial.print(F("Target RPM=")); Serial.println(targetRPM);
    }
  } else Serial.println(F("Commands: arm | v <RPM> | idle | status | scan | probe | clear"));
}
void receiveSerial() {
  for (uint8_t n = 0; n < 32 && Serial.available(); ++n) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (overflow) Serial.println(F("Line too long; discarded"));
      else if (used) { line[used] = 0; processLine(); }
      used = 0; overflow = false;
    } else if (!overflow) {
      if (used < sizeof(line) - 1) line[used++] = ch;
      else overflow = true;
    }
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  if (canBus.begin(MCP_ANY, BITRATE, MCP_CLOCK) != CAN_OK ||
      canBus.setMode(MCP_NORMAL) != CAN_OK) {
    Serial.println(F("MCP2515 init failed; check wiring and crystal"));
    while (true) delay(1000);
  }
  Serial.println(F("Ready, disarmed. Commands: arm | v <RPM> | idle | status"));
  Serial.println(F("Diagnostics: scan | probe | clear. CAN bitrate configured in BITRATE."));
}
void loop() {
  receiveCAN();
  uint32_t now = millis();
  if (scanning && now - scanStartedMs >= 3000) {
    scanning = false; Serial.println(F("Scan complete; send status for bus counters"));
  }
  if ((armed || pending) && (!fresh() || axisError)) {
    stopMotor(); Serial.println(F("Disarmed: heartbeat timeout / axis error"));
  }
  if (pending) {
    if (axisState == 8) {
      pending = false; armed = true; Serial.println(F("Armed at zero RPM"));
    } else if (now - armMs >= 2000) {
      stopMotor(); Serial.println(F("Disarmed: arm timeout"));
    }
  }
  if (armed && axisState != 8) {
    stopMotor(); Serial.println(F("Disarmed: axis left closed loop"));
  }
  receiveSerial();
  if ((armed || pending) && now - commandMs >= 20) {
    commandMs = now;
    if (!velocity(armed ? targetRPM : 0)) {
      stopMotor(); Serial.println(F("Disarmed: transmit failed"));
    }
  }
}
