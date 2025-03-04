package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_ALGAE;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

public class Algae extends SubsystemBase {
    private TalonFX m_Wrist;
    private TalonFX m_Claw;

    private Angle lastDesiredAngle = Degrees.zero();

    @NotLogged
    PositionVoltage positionRequest = new PositionVoltage(0);
    @NotLogged
    VoltageOut voltageRequest = new VoltageOut(0);
    @NotLogged
    MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

    public boolean tryingZero = false;
    public boolean isZero = false;
    public boolean hasAlgaeOverride = false;

    public Algae() {
        m_Wrist = new TalonFX(CONSTANTS_PORTS.ALGAE_WRIST_CAN);
        m_Claw = new TalonFX(CONSTANTS_PORTS.ALGAE_CLAW_CAN);

        m_Wrist.getConfigurator().apply(CONSTANTS_ALGAE.ALGAE_WRIST_CONFIG);
        m_Claw.getConfigurator().apply(CONSTANTS_ALGAE.ALGAE_CLAW_CONFIG);
    }

    public void setClawIntakeMotor(double speed) {
    m_Claw.set(speed);
  }

  public void setWristAngle(Angle setpoint) {
    m_Wrist.setControl(motionRequest.withPosition(setpoint.in(Units.Rotation)));
    lastDesiredAngle = setpoint;
  }

  public AngularVelocity getRotorVelocity() {
    return m_Wrist.getRotorVelocity().getValue();
  }

  public boolean isRotorVelocityZero() {
    return getRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void setWristVoltage(Voltage voltage) {
    m_Wrist.setControl(voltageRequest.withOutput(voltage));
  }

  public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    CONSTANTS_ALGAE.ALGAE_WRIST_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
    CONSTANTS_ALGAE.ALGAE_WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;

    m_Wrist.getConfigurator().apply(CONSTANTS_ALGAE.ALGAE_WRIST_CONFIG);
  }

  public Angle getWristAngle() {
    return m_Wrist.getPosition().getValue();
  }

  public Angle getLastDesiredWristAngle() {
    return lastDesiredAngle;
  }

  public void resetSensorPosition(Angle zeroedPos) {
    m_Wrist.setPosition(zeroedPos);
  }

  public boolean hasAlgae() {
    Current intakeCurrent = m_Claw.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = m_Claw.getVelocity().getValue();
    double intakeAcceleration = m_Claw.getAcceleration().getValueAsDouble();

    Current intakeHasGamePieceCurrent = CONSTANTS_ALGAE.ALGAE_INTAKE_HAS_GP_CURRENT;
    AngularVelocity intakeHasGamePieceVelocity = CONSTANTS_ALGAE.ALGAE_INTAKE_HAS_GP_VELOCITY;

    if (hasAlgaeOverride) {
      return hasAlgaeOverride;
    }

    if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.lte(intakeHasGamePieceVelocity))
        && (intakeAcceleration < 0)) {
      return true;
    } else {
      return false;
    }
  }

  public void setHasAlgaeOverride(boolean hasAlgae) {
    hasAlgaeOverride = hasAlgae;
  }

  public void algaeToggle() {
    this.hasAlgaeOverride = !hasAlgaeOverride;
  }

  public double getClawVoltage() {
    return m_Claw.getMotorVoltage().getValueAsDouble();
  }

  public void setClawVoltage(double voltage) {
    m_Claw.setVoltage(voltage);
  }

  public boolean isAtSetPoint() {
    return (getWristAngle()
        .compareTo(getLastDesiredWristAngle().minus(CONSTANTS_ALGAE.DEADZONE_DISTANCE)) > 0) &&
        getWristAngle().compareTo(getLastDesiredWristAngle().plus(CONSTANTS_ALGAE.DEADZONE_DISTANCE)) < 0;
  }

  public boolean isAtSpecificSetpoint(Angle setpoint) {
    return (getWristAngle()
        .compareTo(setpoint.minus(CONSTANTS_ALGAE.DEADZONE_DISTANCE)) > 0) &&
        getWristAngle().compareTo(setpoint.plus(CONSTANTS_ALGAE.DEADZONE_DISTANCE)) < 0;
  }

  public boolean isAtAnyAlgaeScoringPosition() {
    if (isAtSpecificSetpoint(CONSTANTS_ALGAE.PREP_NET_WRIST_POSITION) ||
        isAtSpecificSetpoint(CONSTANTS_ALGAE.PREP_PROCESSOR_WRIST_POSITION)) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {

  }
}
