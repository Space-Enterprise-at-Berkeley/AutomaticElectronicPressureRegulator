function gas_mass = GasMass(initial_high_pressure, initial_low_pressure, current_high_pressure)

    Constants
    
    total_moles = initial_high_pressure*COPV_volume/(R*temperature)... 
    + initial_low_pressure*ENDO_initial_gas_volume/(R*temperature)...
    + initial_low_pressure*tube_volume/(R*temperature);

    %current_endo_gas_volume = (total_moles-(current_high_pressure * COPV_volume)/(R*temperature))...
    %    *R*temperature/current_low_pressure;
    
    gas_mass = (total_moles-(current_high_pressure * COPV_volume)/(R*temperature))...
        *AIR_MOLAR_MASS;

    

    %current_endo_fluid_volume = ENDO_volume - current_endo_gas_volume; 
end
%PV/RT = n





