PSI_TO_ATM_CONVERSION = 0.068046;
R = .08205;



%initial_high_pressure = 600 * PSI_TO_ATM_CONVERSION;
%initial_low_pressure = 0 * PSI_TO_ATM_CONVERSION;
COPV_volume = 6.8; %Liters
ENDO_initial_gas_volume = 0;
ENDO_volume = 20;
tube_volume = .01;
temperature = 16+273.16;

AIR_MOLAR_MASS = 28.9647/1000;
LIQUID_DENSITY = 1;

% Heat capacity ratio 
lambda = 1.4;