#pragma once

#include "States_generated.h"
enum StateID {
  PRELAUNCH,
  BOOST,
  COAST,
  DROGUE_DESCENT,
  MAIN_DESCENT,
  RECOVERY,
  ABORT,
  NUM_STATES
};

hprc::States stateToTelemState(StateID state);

struct StateData {
  long long currentTime;
  long long deltaTime;
  long long loopCount;
  long long startTime;
  long long lastLoopTime;
};

struct Context;

void initStateData(StateData *data);

void updateStateData(StateData *data);

// ABORT
void *abortInit(StateData const *data);
StateID abortLoop(StateData const *data, Context *ctx, void *localData);

// BOOST
void *boostInit(StateData const *data);
StateID boostLoop(StateData const *data, Context *ctx, void *localData);

// COAST
void *coastInit(StateData const *data);
StateID coastLoop(StateData const *data, Context *ctx, void *localData);

// DROGUE_DESCENT
void *drogueDescentInit(StateData const *data);
StateID drogueDescentLoop(StateData const *data, Context *ctx, void *localData);

// MAIN_DESCENT
void *mainDescentInit(StateData const *data);
StateID mainDescentLoop(StateData const *data, Context *ctx, void *localData);

// PRELAUNCH
void *prelaunchInit(StateData const *data);
StateID prelaunchLoop(StateData const *data, Context *ctx, void *localData);

// RECOVERY
void *recoveryInit(StateData const *data);
StateID recoveryLoop(StateData const *data, Context *ctx, void *localData);
