//Fuel EReg Config 
#pragma once

namespace Config {

    // Controller Constants
    const double p_outer_nominal = 1.0, i_outer_nominal = 0.7e-6, d_outer_nominal = 0.06; // nominal is 4000 -> 500 psi flow
    const double p_inner = 11, i_inner = 3.5e-6, d_inner = 0.10;

    // Flow Parameters
    const float pressureSetpoint = 495;
    const unsigned long loxLead = 200UL * 1000UL; //time in milliseconds

    const float PT_C = 13.482;
    const float PT_M = 1.2407;
    // true = (measured - PT_C) / PT_M
}
