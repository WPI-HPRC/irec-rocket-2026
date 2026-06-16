#include "Context.h"
#include "variant_MARSV21.h"
#include <Arduino.h>

#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>

#include "Packet_generated.h"

#include "boilerplate/Sensors/Impl/ASM330.h"
#include "boilerplate/Sensors/Impl/LIV3F.h"
#include "boilerplate/Sensors/SensorManager/SensorManager.h"

#include "logging.h"

#include "LoRaE22.h"
#include "RadioConfig.h"

SPIClass SENSORS_SPI(SENSORS_SPI_MOSI, SENSORS_SPI_MISO, SENSORS_SPI_SCK);
TwoWire GPS_I2C(GPS_I2C_SDA, GPS_I2C_SCL);
HardwareSerial GPS_SERIAL(GPS_SERIAL_RX, GPS_SERIAL_TX);
TwoWire CONNECTOR_I2C(CONNECTOR_I2C_SDA, CONNECTOR_I2C_SCL);
SPIClass CAMERA_SPI(CAMERA_MOSI, CAMERA_MISO, CAMERA_SCK);
HardwareSerial RADIO_SERIAL(RADIO_SERIAL_RX, RADIO_SERIAL_TX);

Context ctx{
    .asm330 = ASM330(&SENSORS_SPI, SENSORS_ASM_CS),
    .lsm = LSM6(&SENSORS_SPI, SENSORS_LSM_CS),
    .baro = LPS22(&SENSORS_SPI, SENSORS_LPS_CS),
    .mag = LIS2MDL(&SENSORS_SPI, SENSORS_LIS_CS),
    .gps = LIV3F(GPS_SERIAL),
    .radio = LoRaE22(&RADIO_SERIAL, RADIO_M0, RADIO_M1, RADIO_AUX, "KV0R"),
};

SensorManager mgr{
  millis, ctx.baro, ctx.asm330, ctx.lsm, ctx.mag, ctx.gps,
};

StateData data;
void *stateLocalData;

float ekfStartTime = 0;

bool changeSerialPortConfig(RadioConfigTypes::SerialSpeeds baudRate,
                            RadioConfigTypes::ParityConfig parity) {
  // this is safe to call even when the port is not open.
  RADIO_SERIAL.end();

  uint32_t baud = 0;
  uint16_t parityConfig = 0;

  // the radio's baud rates don't follow any pattern over the entire range, so
  // ugly switch statement it is
  switch (baudRate) {
  case RadioConfigTypes::SerialSpeeds::BAUD_1200:
    baud = 1200;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_2400:
    baud = 2400;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_4800:
    baud = 4800;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_9600:
    baud = 9600;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_19200:
    baud = 19200;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_38400:
    baud = 38400;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_57600:
    baud = 57600;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_115200:
    baud = 115200;
    break;
  };
  // this is just easier
  switch (parity) {
  case RadioConfigTypes::ParityConfig::Parity_8N1:
    parityConfig = SERIAL_8N1;
    break;
  case RadioConfigTypes::ParityConfig::Parity_8E1:
    parityConfig = SERIAL_8E1;
    break;
  case RadioConfigTypes::ParityConfig::Parity_8O1:
    parityConfig = SERIAL_8O1;
    break;
  };

  RADIO_SERIAL.begin(baud, parityConfig);

  return true;
}

void radioInit() {
  // build our config
  RadioConfig config;
  config.address = ADDRESS;
  config.networkId = NETWORKID;
  config.encryptionKey = ENCRYPTIONKEY;
  config.parityConfig = PARITYCONFIG;
  config.serialSpeed = SERIALSPEED;
  config.airDataRate = AIRDATARATE;
  config.packetSize = PACKETSIZE;
  config.worMode = WORMODE;
  config.worPeriod = WORPERIOD;
  config.relayMode = RELAYMODE;
  config.destination = DESTINATIONMODE;
  config.txPower = dBm33;
  config.ambientRSSIEnabled = AMBIENTRSSI;
  config.rssiReadingsEnabled = RSSIREADINGS;
  config.listenBeforeTxEnable = LISTENBEFORETX;
  ctx.radio.setConfig(config);
  ctx.radio.setFrequency(FREQUENCY);

  ctx.radio.changeSerialPortCallback(changeSerialPortConfig);
  ctx.radio.setTimeout(2000);

  int8_t code = ctx.radio.init(3);
  ctx.radio.setMode(RadioMode::Normal);

  Log.infoln("radio init done, code: %d", code);
}

// apply remote commadns, one shot are reset after exec
void applyRemoteCommand(hprc::Command command) {
  switch (command) {
  case hprc::Command_ArmFlight:
    ctx.commands.armed = true;
    break;
  case hprc::Command_DeArmFlight:
    ctx.commands.armed = false;
    break;
  case hprc::Command_Reset:
    ctx.commands.resetRequested = true;
    break;
  case hprc::Command_RemoteStartOn:
    ctx.commands.remoteStartActive = true;
    digitalWrite(MOSFET_GATE, HIGH);
    break;
  case hprc::Command_RemoteStartOff:
    ctx.commands.remoteStartActive = false;
    digitalWrite(MOSFET_GATE, LOW);
    break;
  case hprc::Command_StartEstimator:
    ctx.commands.estimatorRequested = true;
    break;
  case hprc::Command_Abort:
    ctx.commands.abortRequested = true;
    break;
  default:
    Log.warningln("radio: ignoring unhandled command %d", (int)command);
    break;
  }
}

void handleRemoteCommands(uint8_t *rxBuff, size_t rxBuffLen) {
  uint8_t messageLength = 0;
  while (ctx.radio.hasMessage() &&
         ctx.radio.getMessage(rxBuff, rxBuffLen, messageLength)) {
    flatbuffers::Verifier verifier(rxBuff, messageLength);
    if (!hprc::VerifyPacketBuffer(verifier)) {
      Log.warningln("radio: dropped malformed packet (%d bytes)", messageLength);
      continue;
    }

    const hprc::Packet *packet = hprc::GetPacket(rxBuff);
    const hprc::RemoteControlCommand *cmd = packet->packet_as_RemoteControl();
    if (cmd == nullptr) {
      continue;
    }

    uint16_t cmdNum = cmd->command_number();
    if (cmdNum == ctx.commands.lastCommandNumber) {
      continue; // already handled this command
    }
    ctx.commands.lastCommandNumber = cmdNum;

    Log.infoln("radio: command #%u -> %s", cmdNum,
               hprc::EnumNameCommand(cmd->command()));
    applyRemoteCommand(cmd->command());
  }
}

void radioLoop() {
  static flatbuffers::FlatBufferBuilder builder;
  static uint8_t rxBuff[1024];
  static uint32_t lastRadioSendTime = 0;
  static uint32_t loopCount = 0;

  // get new messages if there are any
  ctx.radio.update();

  // check for remote commands and apply any we received
  handleRemoteCommands(rxBuff, sizeof(rxBuff));

  if (millis() - lastRadioSendTime >= 200) {
    lastRadioSendTime = millis();
    const auto &asm330_desc = ctx.asm330.get_descriptor();
    const auto &lsm6_desc = ctx.lsm.get_descriptor();
    const auto &baro_desc = ctx.baro.get_descriptor();
    const auto &mag_desc = ctx.mag.get_descriptor();
    const auto &gps_desc = ctx.gps.get_descriptor();

    builder.Clear();
    hprc::SensorsBuilder sensorBuilder(builder);

    static uint32_t lastASM330DataAt = 0;
    if (asm330_desc.getLastUpdated() > lastASM330DataAt) {
      lastASM330DataAt = asm330_desc.getLastUpdated();

      hprc::ASM330Data asm330Data(asm330_desc.data.accel0,
                                  asm330_desc.data.accel1,
                                  asm330_desc.data.accel2, asm330_desc.data.gyr0,
                                  asm330_desc.data.gyr1, asm330_desc.data.gyr2);
      sensorBuilder.add_asm330(&asm330Data);
    }

    static uint32_t lastLSM6DataAt = 0;
    if (lsm6_desc.getLastUpdated() > lastLSM6DataAt) {
      lastLSM6DataAt = lsm6_desc.getLastUpdated();

      hprc::LSM6Data lsm6Data(lsm6_desc.data.accel0, lsm6_desc.data.accel1,
                              lsm6_desc.data.accel2, lsm6_desc.data.gyr0,
                              lsm6_desc.data.gyr1, lsm6_desc.data.gyr2);
      sensorBuilder.add_lsm6(&lsm6Data);
    }

    static uint32_t lastBaroDataAt = 0;
    if (baro_desc.getLastUpdated() > lastBaroDataAt) {
      lastBaroDataAt = baro_desc.getLastUpdated();

      hprc::LPS22Data baroData(baro_desc.data.pressure, baro_desc.data.temp);
      sensorBuilder.add_lps22(&baroData);
    }

    static uint32_t lastMagDataAt = 0;
    if (mag_desc.getLastUpdated() > lastMagDataAt) {
      lastMagDataAt = mag_desc.getLastUpdated();

      hprc::LIS2MDLData magData(mag_desc.data.mag0, mag_desc.data.mag1,
                                mag_desc.data.mag2);
      sensorBuilder.add_lis2mdl(&magData);
    }

    static uint32_t lastGpsDataAt = 0;
    if (gps_desc.getLastUpdated() > lastGpsDataAt) {
      lastGpsDataAt = gps_desc.getLastUpdated();

      hprc::LIV3FData gpsData(gps_desc.data.lat, gps_desc.data.lon,
                              gps_desc.data.alt, gps_desc.data.satellites,
                              gps_desc.data.epochTime);
      sensorBuilder.add_liv3f(&gpsData);
    }

    hprc::Shared sharedData;
    sharedData.mutate_time_from_boot(millis());
    sharedData.mutate_last_command_received(ctx.commands.lastCommandNumber);
    sharedData.mutate_sd_file_no(ctx.logFileIdx);
    sharedData.mutate_loop_count(loopCount);

    auto packetInner = hprc::CreateRocket30KTelemetryPacket(builder, &sharedData, stateToTelemState(ctx.currentState), sensorBuilder.Finish());
    auto packet = hprc::CreatePacket(builder, hprc::PacketUnion_Rocket30KTelemetryPacket, packetInner.Union());

    builder.Finish(packet);
    ctx.radio.sendMessage(builder.GetBufferPointer(), builder.GetSize());
    loopCount++;
  }
}

void sensorsSetup() {
  Log.infoln("Starting MARS board initialization...");
  SENSORS_SPI.begin();

  mgr.sensorInit();

  Log.infoln("\n=== Sensor Initialization Summary ===");
  Log.infoln("Total sensors: %d", mgr.count());
}

void sensorLoop() {
  static unsigned long last_print = 0;
  static int loop_count = 0;

  // Update all sensors through manager
  mgr.loop();

  /*
  if (currentState >= PRELAUNCH) {
      return;
  }
  */

  // static uint32_t lastBaroReadTime = 0;

  // const auto &baro_desc = ctx.baro.get_descriptor();
  // if (baro_desc.getLastUpdated() > lastBaroReadTime) {
  //   lastBaroReadTime = baro_desc.getLastUpdated();
  //   Log.infoln("LPS22 - Pressure: %F hPa, Temp: %F C",
  //              baro_desc.data.pressure, baro_desc.data.temp);
  // }
  

  if (millis() - last_print > 200) {
    digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    last_print = millis();
    loop_count++;

    Log.infoln("=== Loop %d ===", loop_count);

    const auto &asm330_desc = ctx.asm330.get_descriptor();
    const auto &lsm6_desc = ctx.lsm.get_descriptor();
    const auto &baro_desc = ctx.baro.get_descriptor();
    const auto &mag_desc = ctx.mag.get_descriptor();
    const auto &gps_desc = ctx.gps.get_descriptor();

    bool has_data = false;
    // Print LSM6 data
    if (lsm6_desc.getLastUpdated() > 0) {
      Log.infoln("LSM6DSO - Accel: %F, %F, %F | Gyro: %F, %F, %F",
                 lsm6_desc.data.accel0, lsm6_desc.data.accel1,
                 lsm6_desc.data.accel2, lsm6_desc.data.gyr0,
                 lsm6_desc.data.gyr1, lsm6_desc.data.gyr2);
      has_data = true;
    } else {
      Log.warningln("LSM6DSO: No data (timestamp = 0)");
    }

    // Print ASM330 data
    if (asm330_desc.getLastUpdated() > 0) {
      Log.infoln("ASM330- Accel: %F, %F, %F | Gyro: %F, %F, %F",
                 asm330_desc.data.accel0, asm330_desc.data.accel1,
                 asm330_desc.data.accel2, asm330_desc.data.gyr0,
                 asm330_desc.data.gyr1, asm330_desc.data.gyr2);
      has_data = true;
    } else {
      Log.warningln("ASM330: No data (timestamp = 0)");
    }
    // Print LPS22 data
    if (baro_desc.getLastUpdated() > 0) {
      Log.infoln("LPS22 - Pressure: %F hPa, Temp: %F C",
                 baro_desc.data.pressure, baro_desc.data.temp);
      has_data = true;
    } else {
      Log.warningln("LPS22: No data (timestamp = 0)");
    }

    if (mag_desc.getLastUpdated() > 0) {
      Log.infoln("LIS2MDL - Mag: %F, %F, %F", mag_desc.data.mag0,
                 mag_desc.data.mag1, mag_desc.data.mag2);
      has_data = true;
    } else {
      Log.warningln("ICM20948: No data (timestamp = 0)");
    }

    if (gps_desc.getLastUpdated() > 0) {
      Log.infoln("LIV3F - Lat, Lon, Alt: %F, %F, %F | Satellites - %d",
                 gps_desc.data.lat, gps_desc.data.lon, gps_desc.data.alt,
                 gps_desc.data.satellites);
      has_data = true;
    } else {
      Log.warningln("LIV3F: No data (timestamp = 0)");
    }

    Log.infoln("======================\n");
  }
}

void setup() {
  pinMode(MOSFET_GATE, OUTPUT);
  pinMode(ADC_INP4, INPUT);
  digitalWrite(MOSFET_GATE, MOSFET_GATE_DEFAULT_STATE);

  ctx.currentState = PRELAUNCH;
  data = {};

  initStateMap();

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_RED, HIGH);

  Serial.begin(115200);
  // radioInit();

  while (!Serial) {
    delay(10);
  }

  delay(200);

  ctx.ekfLooping = false;
  ctx.sdInitialized = initializeLogging(&ctx);

#if DEBUG_MODE == 0
  Log.begin(LOG_LEVEL_INFO, &ctx.debugLogFile);
#elif DEBUG_MODE == 1
  Log.begin(LOG_LEVEL_INFO, &Serial);
#else
  Log.begin(LOG_LEVEL_TRACE, &Serial);
#endif
  Log.setPrefix([](Print *p, int level) { p->printf("[ %d ] ", millis()); });

  sensorsSetup();

  // NOTE: Run initialization on the first state
  initStateData(&data);
  stateLocalData = (*initFuncs[ctx.currentState])(&data);

  ctx.estimator.init(millis() / 1000.0f, {0, 0, 0}, {0, 0, 0});
  ekfStartTime = millis() / 1000.0f;
  ctx.ekfLooping = true;

  digitalWrite(LED_GREEN, HIGH);
  Log.infoln("=== Starting main loop ===\n");
}

void ekfLoop(Context *ctx) {
  static int state = 0;
  static BLA::Matrix<6, 1> lastCalcTimes = {0, 0, 0, 0, 0, 0};
  static BLA::Matrix<6, 1> runRates = {0.001, 0.03, 0.03, 0.5, 1, 1};

  // seconds since the EKF started
  float t = millis() / 1000.0f - ekfStartTime;

  const auto &asm330_desc = ctx->asm330.get_descriptor();
  const auto &mag_desc = ctx->mag.get_descriptor();

  BLA::Matrix<3, 1> accel = {asm330_desc.data.accel0, asm330_desc.data.accel1,
                             asm330_desc.data.accel2};
  BLA::Matrix<3, 1> gyro = {asm330_desc.data.gyr0, asm330_desc.data.gyr1,
                            asm330_desc.data.gyr2};
  BLA::Matrix<3, 1> mag = {mag_desc.data.mag0, mag_desc.data.mag1,
                           mag_desc.data.mag2};

  if (state == 0) {
    if (t > 5.0f) {
      state = 1;
    }
    ctx->estimator.computeInitialOrientation(ctx->estimator.reorient_asm(accel),
                                             ctx->estimator.reorient_lis(mag),
                                             t);
    lastCalcTimes = {t, t, t, t, t, t};
  } else if (state == 1) {
    if (t > 35.0f) {
      state = 2;
    }
    if (t - lastCalcTimes(0, 0) >= runRates(0, 0)) {
      ctx->estimator.fastGyroProp(ctx->estimator.reorient_asm(gyro), t);
      ctx->estimator.AttekfPredict(t);
      ctx->estimator.runAccelMagUpdate(ctx->estimator.reorient_asm(accel),
                                       ctx->estimator.reorient_lis(mag), t);
      lastCalcTimes(0, 0) = t;
    }
  } else if (state == 2) {
    if (t - lastCalcTimes(0, 0) >= runRates(0, 0)) {
      ctx->estimator.fastGyroProp(ctx->estimator.reorient_asm(gyro), t);
      lastCalcTimes(0, 0) = t;
    }
  }
}

StateID applyCommandEffects(StateID proposedState) {
  if (ctx.commands.estimatorRequested) {
    ctx.commands.estimatorRequested = false;
    ctx.ekfLooping = true;
  }

  // its so over
  if (ctx.commands.abortRequested) {
    ctx.commands.abortRequested = false;
    return ABORT;
  }

  // reset, back to no arm prelaunch
  if (ctx.commands.resetRequested) {
    ctx.commands.resetRequested = false;
    ctx.commands.armed = false;
    return PRELAUNCH;
  }

  return proposedState;
}

void loop() {

  updateStateData(&data);
  StateID newState = (*loopFuncs[ctx.currentState])(&data, &ctx, stateLocalData);
  newState = applyCommandEffects(newState);

  if (ctx.currentState != newState) {
    initStateData(&data);

    if (stateLocalData != nullptr) {
      free(stateLocalData);
    }
    stateLocalData = (*initFuncs[newState])(&data);
    ctx.currentState = newState;
    ctx.debugLogFile.printf("to: %d @ %d\n", newState, millis());
  }

  sensorLoop();

  // NOTE: this is needed for radio
  // radioLoop();

  // there is something up in hte ekf looping func
  // linker gets sad
  if (ctx.ekfLooping) {
    ekfLoop(&ctx);
  }

  loggingLoop(&ctx);
}
