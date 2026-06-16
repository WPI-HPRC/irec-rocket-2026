#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"

void *recoveryInit(StateData const *data) { return nullptr; }

StateID recoveryLoop (StateData const *data, Context* ctx, void *_localData) {

    if(ctx->commands.remoteStartActive) {
        ctx->commands.remoteStartActive = false;
        digitalWrite(MOSFET_GATE, LOW);
    }

    return RECOVERY; 
}
