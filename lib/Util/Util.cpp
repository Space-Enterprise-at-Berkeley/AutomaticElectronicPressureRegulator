#include "Util.h"

namespace Util {
    //TODO implement overflow correction
    long secondsToMillis(long seconds) {
        return seconds * 1000;
    }

    long secondsToMicros(long seconds) {
        return seconds * 1000 * 1000;
    }

    long millisToMicros(long millis) {
        return millis * 1000;
    }
}