package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_ALGAE;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

@Logged
public class Algae extends SubsystemBase {
  private TalonFX m_algaeIntake;

  public static boolean hasAlgaeOverride = false;
  public static boolean stateRun = false;

  public Algae() {
    m_algaeIntake = new TalonFX(CONSTANTS_PORTS.ALGAE_CAN);

    m_algaeIntake.getConfigurator().apply(CONSTANTS_ALGAE.ALGAE_INTAKE_CONFIG);
  }

  public Command intakeAlgae() {
    return runEnd(() -> setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_INTAKE_SPEED), () -> setAlgaeIntakeMotor(0));
  }

  public Command outtakeAlgae() {
    return runEnd(() -> setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_OUTTAKE_SPEED), () -> setAlgaeIntakeMotor(0));
  }

  public void setAlgaeIntakeMotor(double speed) {
    m_algaeIntake.set(speed);
  }

  public boolean hasAlgae() {
    Current intakeCurrent = m_algaeIntake.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = m_algaeIntake.getVelocity().getValue();

    Current intakeHasGamePieceCurrent = CONSTANTS_ALGAE.ALGAE_INTAKE_OCCUPIED_CURRENT;
    AngularVelocity intakeHasGamePieceVelocity = CONSTANTS_ALGAE.ALGAE_INTAKE_OCCUPIED_VELOCITY;

    if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.lte(intakeHasGamePieceVelocity))) {
      return true;
    } else {
      return false;
    }
  }


  public double getAlgaeIntakeVoltage() {
    return m_algaeIntake.getMotorVoltage().getValueAsDouble();
  }

  public void setAlgaeIntakeVoltage(double voltage) {
    m_algaeIntake.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    hasAlgae();

    SmartDashboard.putBoolean("Algae/hasAlgaeOverride", hasAlgaeOverride);
    SmartDashboard.putBoolean("Algae/stateRun", stateRun);
  }
}