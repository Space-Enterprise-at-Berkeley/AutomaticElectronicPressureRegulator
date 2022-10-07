#include "Arduino.h"
#include "HAL.h"
#include "Util.h"

namespace Ducers {
    void init();
    float readInjectorPT();
    float readPressurantPT();
    float readTankPT();
}