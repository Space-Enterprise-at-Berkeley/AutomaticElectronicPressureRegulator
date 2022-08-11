#pragma once

#include <Arduino.h>
// #define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG(val) Serial.print(val)
#define DEBUGLN(val) Serial.println(val)
#define DEBUG_FLUSH() Serial.flush()
#else
#define DEBUG(val)
#define DEBUGLN(val)
#define DEBUG_FLUSH()
#endif

struct Task {
    uint32_t (*taskCall)(void);
    uint32_t nexttime;
    bool enabled = true;
};
