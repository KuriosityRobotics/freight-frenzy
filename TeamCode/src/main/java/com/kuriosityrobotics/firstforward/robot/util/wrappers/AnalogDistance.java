package com.kuriosityrobotics.firstforward.robot.util.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogDistance {
    final AnalogInput analogInput;

    public AnalogDistance(AnalogInput analogInput) {
        this.analogInput = analogInput;
    }

    public double getSensorReading() {
        double voltage_temp_average = 0;

        for (int i = 0; i < 2; i++) {
            voltage_temp_average += analogInput.getVoltage();
        }
        voltage_temp_average /= 2;

        // polynomial moment: 33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4. Used for calculations.
        return (33.9 + -69.5 * (voltage_temp_average) + 62.3 * Math.pow(voltage_temp_average, 2) + -25.4 * Math.pow(voltage_temp_average, 3) + 3.83 * Math.pow(voltage_temp_average, 4)) * 10;
    }
}
