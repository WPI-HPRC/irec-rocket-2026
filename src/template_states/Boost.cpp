#include "debouncer.h"
#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"
//#include "../qmekf.h"

struct BoostData {
    Debouncer accelDebouncer;
    uint32_t lastAccelReadingTime;
};

void *boostInit (StateData const *data) {
    BoostData *localData = static_cast<BoostData *>(malloc(sizeof(BoostData)));
    localData->accelDebouncer = Debouncer(20);
    localData->lastAccelReadingTime = 0;

    // Enable live camera

    return localData;
}

StateID boostLoop (StateData const *data, Context* ctx, void *_localData) {
    /*
    - Poll acceleration data from ctx
    - Check acceleration to detect coast stage
    - Check if maximum boost time is exceeded
    - Check if need to abort
    - Update sensor data and ctx for next iteration?
    */
    BoostData *localData = static_cast<BoostData *>(_localData);
    
    const auto &accel_desc = ctx->asm330.get_descriptor();

    if(!ctx->commands.remoteStartActive) {
        ctx->commands.remoteStartActive = true;
        digitalWrite(MOSFET_GATE, HIGH);
    }

    if(ctx->boostStartTime != data->startTime) {
        ctx->boostStartTime = data->startTime;
    }

    if (accel_desc.getLastUpdated() != localData->lastAccelReadingTime) {
        localData->lastAccelReadingTime = accel_desc.getLastUpdated();
        float totalAccel = accel_desc.data.accel0 * accel_desc.data.accel0 +
                        accel_desc.data.accel1 * accel_desc.data.accel1 +
                        accel_desc.data.accel2 * accel_desc.data.accel2;
        if (localData->accelDebouncer.update(abs(accel_desc.data.accel0) < 2, millis())) {
            return COAST;
        }
    }

    //const auto &accel_desc = ctx->accel.get_descriptor();
    //StateEstimator state_estimator = ctx->estimator;
    // const auto acc_vec = ctx->estimator.get_accel_prev();
    // if (localData->accelDebouncer.update(abs(acc_vec(0, 0)) < COAST_THRESHOLD, data->currentTime) ||  data->currentTime > 2000) {
    //     // check that acceleration up is less than threshold, or
    //     // current time > 2000
    //     return COAST;
    // }

    if(data->currentTime > BOOST_TIMEOUT) {
        return COAST;
    }

    return BOOST;
}
