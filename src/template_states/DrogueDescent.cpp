#include "debouncer.h"
#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"
#include "StateMachineConstants.h"

struct DrogueDescentData {
    Debouncer velDebouncer;
};

void *drogueDescentInit(StateData const *data) {
    DrogueDescentData *localData = static_cast<DrogueDescentData *>(malloc(sizeof(DrogueDescentData)));
    localData->velDebouncer = Debouncer(20);

    return localData;
}

StateID drogueDescentLoop (StateData const *data, Context* ctx, void *_localData) {
    DrogueDescentData *localData = static_cast<DrogueDescentData *>(_localData);

    // under main descent if velocity down is between 16 to 30 fps
    // this is abt 5m/s to 9m/s
    const auto vel_vec = ctx->estimator.get_vel_ned();
    // velocity in the vertical direction is negative
    // going down will be a positive value
    if(localData->velDebouncer.update(abs(vel_vec(2, 0)) > MAIN_MIN_VEL &&
                                      abs(vel_vec(2, 0)) < MAIN_MAX_VEL,
                                      data->currentTime)) {
        return MAIN_DESCENT;
    }

    if(data->currentTime > DROGUE_DESCENT_TIMEOUT) {
        return MAIN_DESCENT;
    }

    return DROGUE_DESCENT;
}
