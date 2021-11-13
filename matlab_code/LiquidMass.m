function liquid_mass = LiquidMass(gas_mass, current_low_pressure)
    Constants
    
    gas_volume = gas_mass / AIR_MOLAR_MASS*R*temperature / current_low_pressure;
    liquid_volume = ENDO_volume - gas_volume;
    liquid_mass = liquid_volume * LIQUID_DENSITY;
end