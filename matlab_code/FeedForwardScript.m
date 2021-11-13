clear
Constants
times = linspace(0, 12, 500);
high_pressure_tank_ideal_pressures = arrayfun(@CurrentHighPressureCalc, times);
motor_ideal_angles = arrayfun(@MotorAngleCalc, high_pressure_tank_ideal_pressures);

tiledlayout(2,1)

nexttile
plot(times, high_pressure_tank_ideal_pressures);
title("High Pressure Tank Pressures vs Flow Time");

nexttile
plot(times, motor_ideal_angles);
title("Motor Ideal Angle vs Flow Time");