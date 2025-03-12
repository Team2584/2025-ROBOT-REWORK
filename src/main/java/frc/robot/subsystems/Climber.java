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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_CLIMB;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

@Logged
public class Climber extends SubsystemBase {
    private TalonFX m_climb;

    private Angle lastTargetPosition;

    @NotLogged
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public Climber() {
        lastTargetPosition = Units.Degrees.of(0);
        m_climb = new TalonFX(CONSTANTS_PORTS.CLIMB_CAN);

        m_climb.getConfigurator().apply(CONSTANTS_CLIMB.CLIMBER_CONFIG);
        // TODO: set climb position to zero at start of robot intialise (make that state
        // or whatever)
    }

      
    public Command liftRobot() {
        return runEnd(() -> setVoltage(9), () -> setVoltage(0));
    }

    /**
     * Sets the climb to lower the robot at a contant speed
     */
    public Command lowerRobot() {
        return runEnd(() -> setVoltage(-9), () -> setVoltage(0));
    }

    public void setClimberMotorVelocity(double velocity) {
        m_climb.set(velocity);
    }

    public Angle getClimberPosition() {
        return m_climb.getPosition().getValue();
    }

    public void setPosition(Angle angle) {
        m_climb.setControl(new PositionVoltage(angle.in(Units.Rotations)));
        lastTargetPosition = angle;
    }

    public Angle getLastTargetPosition() {
        return lastTargetPosition;
    }

    public void setNeutral() {
        m_climb.setControl(new NeutralOut());
    }

    public void resetClimbPosition(Angle setpoint) {
        m_climb.setPosition(setpoint.in(Rotations));
    }

    public boolean isClimbDeployed() {
        return getClimberPosition().gte(CONSTANTS_CLIMB.MAX_POSITION.minus(CONSTANTS_CLIMB.POSITION_TOLERANCE));
    }

    public boolean isClimbRetracted() {
        return getClimberPosition().lte(CONSTANTS_CLIMB.MIN_POSITION.plus(CONSTANTS_CLIMB.POSITION_TOLERANCE));
    }

    public void setVoltage(double Volts) {
        m_climb.setControl(voltageRequest.withOutput(Volts));
    }

    @Override
    public void periodic() {
    }
}
