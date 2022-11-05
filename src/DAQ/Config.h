#define FUEL_TANK_EREG_ADDR 25
#define LOX_TANK_EREG_ADDR 26
#define FUEL_INJECTOR_EREG_ADDR 27
#define LOX_INJECTOR_EREG_ADDR 28
#define AC_EREG_ADDR 21
#define DAQ_EREG_ADDR 22
#define THERMOCOUPLES_DAQ_TO_DASHBOARD 70
#define LOAD_CELLS_DAQ_TO_DASHBOARD 70
#define FLOW_ABORT_ID 201
#pragma once

namespace Config {
    
    // TC Aborts
    const float temperatureThreshold = 20; // hotfire: 200
    const int tempBufferSize = 5;
    const unsigned long minTimePerStoredTemp = 100; // time in millis 
    const int tempNumberThreshold = 3; // abort if >= numberThreshold samples are greater than temperatureThreshold
    const int numberOfTC = 3;

};
