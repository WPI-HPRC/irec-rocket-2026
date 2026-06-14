#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"

void *abortInit (StateData const *data) { return nullptr; }

StateID abortLoop (StateData const *data, Context *ctx, void *localData) {
    return ABORT; // this is what code from last year was doing, may need to do more
}
