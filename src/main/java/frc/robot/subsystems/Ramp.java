package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_RAMP;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

@Logged
public class Ramp extends SubsystemBase {
    private TalonFX m_ramp;

    private Angle lastTargetPosition;

    @NotLogged
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public Ramp() {
        lastTargetPosition = Units.Degrees.of(0);
        m_ramp = new TalonFX(CONSTANTS_PORTS.RAMP_CAN);

        m_ramp.getConfigurator().apply(CONSTANTS_RAMP.RAMP_CONFIG);
    }

    public void setRampMotorVelocity(double velocity) {
        m_ramp.set(velocity);
    }

    public Angle getRampPosition() {
        return m_ramp.getPosition().getValue();
    }

    public void setPosition(Angle angle) {
        m_ramp.setControl(new PositionVoltage(angle.in(Units.Rotations)));
        lastTargetPosition = angle;
    }

    public Angle getLastTargetPosition() {
        return lastTargetPosition;
    }

    public void setNeutral() {
        m_ramp.setControl(new NeutralOut());
    }

    public void resetRampPosition(Angle setpoint) {
        m_ramp.setPosition(setpoint.in(Rotations));
    }

    public boolean isRampDown() {
        return getRampPosition().gte(CONSTANTS_RAMP.MAX_POSITION.minus(CONSTANTS_RAMP.POSITION_TOLERANCE));
    }

    public boolean isRampUp() {
        return getRampPosition().lte(CONSTANTS_RAMP.MIN_POSITION.plus(CONSTANTS_RAMP.POSITION_TOLERANCE));
    }

    public void setVoltage(double Volts) {
        m_ramp.setControl(voltageRequest.withOutput(Volts));
    }

    @Override
    public void periodic() {

    }
}
