package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;
import frc.robot.CONSTANTS.CONSTANTS_WRIST;

@Logged
public class Wrist extends SubsystemBase {
    private TalonFX m_wrist;

    private Angle lastDesiredAngle = Degrees.zero();

    @NotLogged
    VoltageOut voltageRequest = new VoltageOut(0);
    @NotLogged
    MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

    public Wrist() {
        m_wrist = new TalonFX(CONSTANTS_PORTS.WRIST_CAN);

        m_wrist.getConfigurator().apply(CONSTANTS_WRIST.WRIST_CONFIG);
    }

    public void setWristAngle(Angle setpoint) {
        m_wrist.setControl(motionRequest.withPosition(setpoint.in(Units.Rotation)));
        lastDesiredAngle = setpoint;
    }

    public AngularVelocity getMotorVelocity() {
        return m_wrist.getRotorVelocity().getValue();
    }

    public boolean isMotorVelocityZero() {
        return getMotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
    }

    public void setVoltage(Voltage voltage) {
        m_wrist.setControl(voltageRequest.withOutput(voltage));
    }

    public void setWristMotorVelocity(double velocity) {
        m_wrist.set(velocity);
    }

    public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
        CONSTANTS_WRIST.WRIST_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
        CONSTANTS_WRIST.WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;

        m_wrist.getConfigurator().apply(CONSTANTS_WRIST.WRIST_CONFIG);
    }

    public Angle getPivotAngle() {
        return m_wrist.getPosition().getValue();
    }

    public Angle getLastDesiredPivotAngle() {
        return lastDesiredAngle;
    }

    public void resetSensorPosition(Angle zeroedPos) {
        m_wrist.setPosition(zeroedPos);
    }

    public boolean isAtSetPoint() {
        return (getPivotAngle()
                .compareTo(getLastDesiredPivotAngle().minus(CONSTANTS_WRIST.DEADZONE_DISTANCE)) > 0) &&
                getPivotAngle().compareTo(getLastDesiredPivotAngle().plus(CONSTANTS_WRIST.DEADZONE_DISTANCE)) < 0;
    }

    public boolean isAtSpecificSetpoint(Angle setpoint) {
        return (getPivotAngle()
                .compareTo(setpoint.minus(CONSTANTS_WRIST.DEADZONE_DISTANCE)) > 0) &&
                getPivotAngle().compareTo(setpoint.plus(CONSTANTS_WRIST.DEADZONE_DISTANCE)) < 0;
    }

    public boolean isAtAnyAlgaeScoringPosition() {
        if (isAtSpecificSetpoint(CONSTANTS_WRIST.PIVOT_ALGAE_NET)) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist/PivotAngle (deg)", getPivotAngle().in(Degrees));
        SmartDashboard.putNumber("Wrist/Setpoint (deg)", lastDesiredAngle.in(Degrees));

    }
}