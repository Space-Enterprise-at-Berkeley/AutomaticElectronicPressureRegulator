function motor_angle = MotorAngleCalc(current_high_pressure)
    Constants;
     %valid for choked flow
    Cv_at_angle = ideal_gas_mass_flow_rate/(sqrt(lambda*current_high_pressure*(2/(lambda+1))^((lambda+1)/(lambda-1))));
    motor_angle = Cv_to_Angle(Cv_at_angle);
end