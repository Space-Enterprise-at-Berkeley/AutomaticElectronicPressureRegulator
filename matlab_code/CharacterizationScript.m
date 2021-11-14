clear;
Constants

table = readtable("cv_char4.xlsx");

table_length = length(table.Time);

scaled_time = table.Time;
scaled_time = scaled_time./1000.0-table.Time(1);

initial_high_pressure_values = zeros(table_length, 1)+735;%zeros(table_length, 1)+table.HP(1);
initial_low_pressure_values = zeros(table_length, 1)+0;%zeros(table_length, 1)+table.LP(1);

air_mass = arrayfun(@GasMass, initial_high_pressure_values, initial_low_pressure_values, table.HP);
liquid_mass = arrayfun(@LiquidMass, air_mass, table.LP);

tiledlayout(2,3)
nexttile
plot(scaled_time, air_mass);
title('Seconds vs Mass Air in Endo Tank Kg');

nexttile
plot(scaled_time, liquid_mass);
title('Seconds vs Mass Water in Endo Tank Kg');

air_mass(end+1) = air_mass(end);
time_scale = scaled_time;
time_scale(end+1) = time_scale(end);

coefficients = polyfit(time_scale, air_mass, 3);
numFitPoints = table_length; % Enough to make the plot look continuous.
xFit = linspace(scaled_time(1), scaled_time(end), numFitPoints);
yFit = polyval(coefficients, xFit);

nexttile
plot(xFit, yFit, 'r-', 'LineWidth', 2);
title('Line of best fit for Seconds vs Mass kg');




air_mass = yFit;
%dM_dt is mass flow rate in kilograms per second
dM_dt = zeros(table_length, 1);

for n = 1 : table_length
    delta_t = time_scale(n+1)-time_scale(n);
    if delta_t == 0
        dM_dt(n) = 0;
    else
        dM_dt(n) = (air_mass(n+1)-air_mass(n))/delta_t;
    end
end

nexttile
%plot(scaled_time, dM_dt);
%title('Seconds vs Mass Flow Rate Air kg/sec');
plot(scaled_time, table.HP)
hold on
plot(scaled_time, table.LP)
title('Seconds vs High Pressure and Low Pressure');



nexttile
plot(scaled_time, table.Angle);
title('Seconds vs Valve Angle');

%average_value_dM_dt = (scaled_time(end) - scaled_time(1)) * coefficients(1)/2.0+coefficients(2);
%fprintf("Average Mass Flow Rate in kg/seconds: %f", average_value_dM_dt);
    

Cv_choked = zeros(table_length, 1);
%If flow is choked:
for n = 1 : table_length
    hp = table.HP(n);
    lp = table.LP(n);
    critical_pressure = (2/(lambda+1))^(lambda/(lambda-1)) * hp;

    
    if (lp<critical_pressure)
        %choked flow
        % From https://en.wikipedia.org/wiki/Choked_flow
        % mass_flow_rate = Cd * A * sqrt(lambda*hp*(2/(lambda+1))^((lambda+1)/(lambda-1)));
        % replace Cd*A with Cv of valve
        rho = AIR_MOLAR_MASS*hp/(R*temperature); %fill out rho

        %mass_flow_rate = Cd * A * sqrt(lambda*rho*hp*(2/(lambda+1))^((lambda+1)/(lambda-1)));
        Cv_choked(n) = dM_dt(n) / (sqrt(lambda*rho*hp*(2/(lambda+1))^((lambda+1)/(lambda-1))));
    else
        disp("non-choked-flow");
    end
end
Cv_choked(Cv_choked == 0) = NaN;
nexttile
plot(table.Angle, Cv_choked);
title('Valve Angle vs Cv_choked');





