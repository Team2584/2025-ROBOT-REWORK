// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

public class LED extends SubsystemBase {
    private final CANdle m_candle = new CANdle(CONSTANTS_PORTS.CANDLE_CAN, "rio");

    private final Algae m_algae;
    private final Coral m_coral;

    // private String currentColor;

    public LED(RobotContainer RC) {
        m_algae = RC.getAlgae();
        m_coral = RC.getCoral();

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = true;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    // Wrappers
    public double getVbat() {
        return m_candle.getBusVoltage();
    }

    public double get5V() {
        return m_candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return m_candle.getCurrent();
    }

    public double getTemperature() {
        return m_candle.getTemperature();
    }

    public void configBrightness(double percent) {
        m_candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        m_candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        m_candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        m_candle.configStatusLedState(offWhenActive, 0);
    }

    // public void turnGreen() {
    // currentColor = "green";
    // }

    // public Command turnGreenCommand() {
    // return runOnce(()->turnGreen());
    // }

    // public void turnOrange() {
    // currentColor = "orange";
    // }

    // public Command turnOrangeCommand() {
    // return runOnce(()->turnOrange());
    // }

    // public void turnOff() {
    // currentColor = null;
    // }

    // public Command turnOffCommand() {
    // return runOnce(()->turnOff());
    // }

    @Override
    public void periodic() {
        if (m_algae.hasAlgae() && m_coral.coralLoaded()) {
            m_candle.setLEDs(0, 0, 255); // BLUE
        } else if (m_algae.hasAlgae()) {
            m_candle.setLEDs(0, 255, 0); // GREEN
        } else if (m_coral.coralLoaded()) {
            m_candle.setLEDs(255, 165, 0); // ORANGE
        } else {
            m_candle.setLEDs(0, 0, 0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Not sure what to put here, fix as needed :) - Shuntao
    }
}
