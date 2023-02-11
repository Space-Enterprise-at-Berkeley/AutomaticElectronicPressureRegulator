#pragma once

#include <Arduino.h>
// #define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG(val) Serial.print(val)
#define DEBUGLN(val) Serial.println(val)
#define DEBUG_FLUSH() Serial.flush()
#define DEBUGF Serial.printf
#else
#define DEBUG(val)
#define DEBUGLN(val)
#define DEBUG_FLUSH()
#define DEBUGF(...)
#endif



struct Task {
    uint32_t (*taskCall)(void);
    uint32_t nexttime;
    bool enabled = true;

    Task(uint32_t (*taskCallin)(void), uint32_t nexttimein, bool enabledin):
        taskCall(taskCallin), nexttime(nexttimein), enabled(enabledin) {}

    Task(uint32_t (*taskCallin)(void), uint32_t nexttimein):
        taskCall(taskCallin), nexttime(nexttimein), enabled(true) {}
};


