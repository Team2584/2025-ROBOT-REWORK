package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

@Logged
public class Elevator extends SubsystemBase {
    private TalonFX m_Follower_Left;
    private TalonFX m_Leader_Right;
    private DigitalInput elevatorZeroLimit;

    private Distance lastDesiredPosition;

    Distance currentLeftPosition = Units.Inches.of(0);
    Distance currentRightPosition = Units.Inches.of(0);

    @NotLogged
    PositionVoltage positionRequest;
    @NotLogged
    VoltageOut voltageRequest = new VoltageOut(0);

    public boolean tryingZero = false;
    public boolean isZero = false;

    @NotLogged
    MotionMagicVoltage motionRequest;

    public Elevator() {
        this.m_Leader_Right = new TalonFX(CONSTANTS_PORTS.ELEVATOR_RIGHT_CAN);
        this.m_Follower_Left = new TalonFX(CONSTANTS_PORTS.ELEVATOR_LEFT_CAN);
        this.elevatorZeroLimit = new DigitalInput(CONSTANTS_PORTS.ELEVATOR_LIMIT_CHANNEL);

        lastDesiredPosition = Units.Inches.of(0);
        voltageRequest = new VoltageOut(0);
        motionRequest = new MotionMagicVoltage(0);

        m_Leader_Right.getConfigurator().apply(CONSTANTS_ELEVATOR.ELEVATOR_CONFIG);

        // m_Follower_Left.setControl(new Follower(CONSTANTS_PORTS.ELEVATOR_RIGHT_CAN,
        // true));
    }
    

    public Distance getLastDesiredPosition() {
        return lastDesiredPosition;
    }

    public Distance getElevatorPosition() {
        return Units.Inches.of(m_Leader_Right.getPosition().getValueAsDouble());
    }

    public boolean isAtSetPoint() {
        return (getElevatorPosition()
                .compareTo(getLastDesiredPosition().minus(CONSTANTS.CONSTANTS_ELEVATOR.DEADZONE_DISTANCE)) > 0) &&
                getElevatorPosition()
                        .compareTo(getLastDesiredPosition().plus(CONSTANTS.CONSTANTS_ELEVATOR.DEADZONE_DISTANCE)) < 0;
    }

    public boolean isAtSpecificSetpoint(Distance setpoint) {
        Distance currentPosition = getElevatorPosition();
        Distance lowerBound = setpoint.minus(CONSTANTS.CONSTANTS_ELEVATOR.DEADZONE_DISTANCE);
        Distance upperBound = setpoint.plus(CONSTANTS.CONSTANTS_ELEVATOR.DEADZONE_DISTANCE);
        return currentPosition.compareTo(lowerBound) >= 0 && currentPosition.compareTo(upperBound) <= 0;
    }

    public boolean isAtAnyCoralScoringPosition() {
        if (isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L1) ||
                isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L2) ||
                isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L3) ||
                isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4)) {
            return true;
        }
        return false;
    }

    public boolean isAtAnyAlgaeScoringPosition() {
        if (isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_NET) ||
                isAtSpecificSetpoint(CONSTANTS_ELEVATOR.HEIGHT_PROCESSOR)) {
            return true;
        }
        return false;
    }

    public boolean getZeroLimit() {
        return !elevatorZeroLimit.get(); // returns true if the limit switch is touched
    }

    public void homeElevator() {
        if (getZeroLimit()) {
          resetSensorPosition(Units.Inches.of(0));
        } else {
          return;
        }
      }

    public AngularVelocity getMotorVelocity() {
        return m_Leader_Right.getRotorVelocity().getValue();
    }

    public void setCoastMode(Boolean coastMode) {
        if (coastMode) {
            m_Leader_Right.getConfigurator().apply(CONSTANTS_ELEVATOR.COAST_MODE_CONFIGURATION);
            m_Follower_Left.getConfigurator().apply(CONSTANTS_ELEVATOR.COAST_MODE_CONFIGURATION);
        } else {
            m_Leader_Right.getConfigurator().apply(CONSTANTS_ELEVATOR.ELEVATOR_CONFIG);
            m_Follower_Left.getConfigurator().apply(CONSTANTS_ELEVATOR.ELEVATOR_CONFIG);
        }
    }

    public boolean isMotorVelocityZero() {
        return getMotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
    }

    public void setPosition(Distance height) {
        m_Leader_Right.setControl(motionRequest.withPosition(inchesToRotations(height.in(Units.Inches))));
        m_Follower_Left.setControl(new Follower(CONSTANTS_PORTS.ELEVATOR_RIGHT_CAN, true));
    }


    public void setNeutral() {
        m_Leader_Right.setControl(new NeutralOut());
        m_Follower_Left.setControl(new NeutralOut());
    }

    public void setVoltage(Voltage voltage) {
        m_Leader_Right.setControl(voltageRequest.withOutput(voltage));
        m_Follower_Left.setControl(new Follower(CONSTANTS_PORTS.ELEVATOR_RIGHT_CAN, true));
    }

    public void resetSensorPosition(Distance setpoint) {
        m_Leader_Right.setPosition(setpoint.in(Inches));
        m_Follower_Left.setPosition(setpoint.in(Inches));
    }

    public void setHardLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
        CONSTANTS_ELEVATOR.ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
        CONSTANTS_ELEVATOR.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;

        m_Leader_Right.getConfigurator().apply(CONSTANTS_ELEVATOR.ELEVATOR_CONFIG);
        m_Follower_Left.getConfigurator().apply(CONSTANTS_ELEVATOR.ELEVATOR_CONFIG);
    }

    private double rotationsToInches(double rotations) {
        return rotations * (Math.PI * CONSTANTS_ELEVATOR.ELEVATOR_PULLEY_PITCH_DIAMETER.in(Inches));
    }

    private double inchesToRotations(double heightInches) {
        return (heightInches / (Math.PI * CONSTANTS_ELEVATOR.ELEVATOR_PULLEY_PITCH_DIAMETER.in(Inches)));
    }

    @Override
    public void periodic() {
        if (getZeroLimit()) {
            isZero = true;
            resetSensorPosition(Units.Inches.of(0));
        }
        SmartDashboard.putBoolean("Elevator/isZero", isZero);
        SmartDashboard.putNumber("Elevator/height (in)",
                rotationsToInches(m_Leader_Right.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Elevator/target (in)", lastDesiredPosition.in(Units.Inches));
    }
}