#include "debouncer.h"
#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"

struct MainDescentData {
    Debouncer velDebouncer;
};

void *mainDescentInit(StateData const *data) {
    MainDescentData *localData = static_cast<MainDescentData *>(malloc(sizeof(MainDescentData)));
    localData->velDebouncer = Debouncer(20);

    return localData;
}

StateID mainDescentLoop (StateData const *data, Context* ctx, void *_localData) {
    MainDescentData *localData = static_cast<MainDescentData *>(_localData);
    // when we land we are no longer falling
    // should see vertical velocity that is zero

    const auto vel_vec = ctx->estimator.get_vel_ned();
    // check if |vertical velocity| less than 2 m/s
    if(localData->velDebouncer.update(abs(vel_vec(2, 0)) < 2, millis())) {  
        return RECOVERY;
    }

    return MAIN_DESCENT;
}
