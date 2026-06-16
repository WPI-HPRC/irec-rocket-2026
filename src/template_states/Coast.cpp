#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"

void *coastInit (StateData const *data) { return nullptr; }

StateID coastLoop (StateData const *data, Context* ctx, void *_localData) {
    /*
    - Poll acceleration data from ctx
    - Check acceleration to detect drouge deployment
    - Check if maximum coast time is exceeded
    - Check if need to abort
    - Update sensor data and ctx for next iteration?
    */

    //const auto &baro_desc = ctx->baro.get_descriptor();
    //if (baro_desc.getLastUpdated() != data->lastBaroReadingTime) {
    //    data->lastBaroReadingTime = baro_desc.getLastUpdated();
    //    double currentAltitiude = solveAltitude(baro_desc.data.pressure);
    //    if(data->baroDebouncer.update((currentAltitiude - prevAltitude) < 0 ,millis()) && airBrakesDone) {
    //        return DROGUE_DESCENT;
    //    }
    //    prevAltitude = currentAltitiude;
    //}
    const auto vel_vec = ctx->estimator.get_vel_ned();
    // apogee: vertical NED velocity has dropped to near zero (between -0.2 and 0.2)
    if(vel_vec(2, 0) < 0.2 && vel_vec(2, 0) > -0.2) {
        return DROGUE_DESCENT;
    }

    return COAST;
}
