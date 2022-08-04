#pragma once

namespace Config {

    #define MAX_SPD 255
    #define MIN_SPD -255
    // #define STATIC_SPD 60
    #define STATIC_SPD 0
    //old max angle = 1296
    //3200*48/26
    #define MAX_ANGLE 363
    #define MIN_ANGLE 0

    const double p_outer = 1.50, i_outer = 2.25e-6, d_outer = 0.125;
    const double p_inner = 11.5, i_inner = 1.5e-6, d_inner = 0.35e-6;
}
