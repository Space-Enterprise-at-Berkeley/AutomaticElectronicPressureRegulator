//Fuel EReg Config 
#pragma once

namespace Config {

    // Controller Constants
    const double p_outer_nominal = 1.0, i_outer_nominal = 0.7e-6, d_outer_nominal = 0.06; // nominal is 4000 -> 500 psi flow
    const double p_inner = 11, i_inner = 3.5e-6, d_inner = 0.1;

    // Flow Parameters
    const float pressureSetpoint = 490;
    const unsigned long loxLead = 105UL * 1000UL; //time in milliseconds

    const float LOW_PT_C = 13.482;
    const float LOW_PT_M = 1.2407;

    const float HIGH_PT_C = 5.0;
    const float HIGH_PT_M = 0.7973;
    // true = (measured - LOW_PT_C) / LOW_PT_M
}
