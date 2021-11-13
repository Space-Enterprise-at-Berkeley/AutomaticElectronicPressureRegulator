PSI_TO_ATM_CONVERSION = 0.068046;
R = 0.082057 *14.6959;

%initial_high_pressure = 600 * PSI_TO_ATM_CONVERSION;
%initial_low_pressure = 0 * PSI_TO_ATM_CONVERSION;
COPV_volume = 6.8; %Liters
ENDO_initial_gas_volume = 0.0;
ENDO_volume = 18.0;
tube_volume = .01;
temperature = 18+273.16;

AIR_MOLAR_MASS = 28.9647/1000;
LIQUID_DENSITY = 1.141;

% Heat capacity ratio 
lambda = 1.4;


ideal_endo_pressure = 600;
%kg/s
ideal_flow_rate = 0.95;
initial_ideal_COPV_pressure = 4500;
initial_ideal_endo_pressure = 0;

%pv=nrt
%n=pv/rt
%change in volume of fluid = ideal_flow_rate = kg/s
ideal_volume_fluid_flow_rate = ideal_flow_rate/LIQUID_DENSITY;
ideal_gas_mol_flow_rate = ideal_volume_fluid_flow_rate*ideal_endo_pressure/(R*temperature);
ideal_gas_mass_flow_rate = ideal_gas_mol_flow_rate / AIR_MOLAR_MASS;

