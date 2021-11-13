function current_high_pressure = CurrentHighPressureCalc(time)
    Constants
    total_moles = initial_ideal_COPV_pressure*COPV_volume/(R*temperature)... 
    + initial_ideal_endo_pressure * ENDO_initial_gas_volume/(R*temperature);

    current_low_pressure_gas_volume = ideal_flow_rate*time;
    %pv = nrt
    high_pressure_moles = total_moles - current_low_pressure_gas_volume*ideal_endo_pressure/(R*temperature);
    current_high_pressure = high_pressure_moles*R*temperature/COPV_volume;
    
   
end