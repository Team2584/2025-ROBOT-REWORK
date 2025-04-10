package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

public class LED extends SubsystemBase {
    private final CANdle m_candle = new CANdle(CONSTANTS_PORTS.CANDLE_CAN, "rio");

    private final int[] rgb;

    // private String currentColor;

    public LED(RobotContainer RC) {
        rgb = new int[3];

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

    public void setColor(int r, int g, int b) {
        rgb[0] = r;
        rgb[1] = g;
        rgb[2] = b;
    }

    @Override
    public void periodic() {
        m_candle.setLEDs(rgb[0], rgb[1], rgb[2]);
    }

    @Override
    public void simulationPeriodic() {
    }
}
