#include "debouncer.h"
#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"
#include "Arduino.h"
#include "StateMachineConstants.h"
#include "config.h"
#include "logging.h"

struct PrelaunchData {
  Debouncer accelDebouncer;
  uint32_t  lastAccelReadingTime;
};

void *prelaunchInit(StateData const *data) {
  PrelaunchData *localData = static_cast<PrelaunchData *>(malloc(sizeof(PrelaunchData)));
  localData->accelDebouncer = Debouncer(20);
  localData->lastAccelReadingTime = 0;

  return localData;
}

StateID prelaunchLoop(StateData const *data, Context *ctx, void *_localData) {
  PrelaunchData *localData = static_cast<PrelaunchData *>(_localData);
  //static bool ComuteInitialOrientationThisLoop = false;

  const auto &accel_desc = ctx->asm330.get_descriptor();
  BLA::Matrix<3, 1> accel = {accel_desc.data.accel0, accel_desc.data.accel1,
                             accel_desc.data.accel2};

  const auto &mag_desc = ctx->mag.get_descriptor();
  BLA::Matrix<3, 1> mag = {mag_desc.data.mag0, mag_desc.data.mag1,
                           mag_desc.data.mag2};

  const auto &gps_desc = ctx->gps.get_descriptor();
  // BLA::Matrix<3, 1> gps = {gps_desc.data.ecefX, gps_desc.data.ecefY,
  //                          gps_desc.data.ecefZ};

  // Stay in PRELAUNCH if not armed
  if(!ctx->commands.armed) {
    return PRELAUNCH;
  }

  // Estimator init + ekfLooping are handled in setup() (see ekfLoop), matching
  // the canard implementation, so nothing to do here on the pad.

  /*
  - Poll acceleration data from ctx
  - Check acceleration to detect launch
  - Check if need to abort
  - Update sensor data and ctx for next iteration?
  */
  if (accel_desc.getLastUpdated() != localData->lastAccelReadingTime) {
    localData->lastAccelReadingTime = accel_desc.getLastUpdated();
    if (localData->accelDebouncer.update(accel_desc.data.accel0 > LAUNCH_TRHESHOLD, millis())) {
      return BOOST;
    }
  }

  // const auto acc_vec = ctx->estimator.get_accel_prev();
  // // check acceleration in vertical direction is greater than threshold
  // if (localData->accelDebouncer.update(abs(acc_vec(0, 0)) > LAUNCH_TRHESHOLD, data->currentTime)) {
  //   return BOOST;
  // }

  return PRELAUNCH;
}
