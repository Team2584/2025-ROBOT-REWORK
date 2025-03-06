package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public void setAlgaeIntakeMotor(double speed) {
    m_algaeIntake.set(speed);
  }

  public boolean hasAlgae() {
    Current intakeCurrent = m_algaeIntake.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = m_algaeIntake.getVelocity().getValue();

    Current intakeHasGamePieceCurrent = CONSTANTS_ALGAE.ALGAE_INTAKE_OCCUPIED_CURRENT;
    AngularVelocity intakeHasGamePieceVelocity = CONSTANTS_ALGAE.ALGAE_INTAKE_OCCUPIED_VELOCITY;

    if (hasAlgaeOverride) {
      return hasAlgaeOverride;
    }

    if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.lte(intakeHasGamePieceVelocity))) {
      return true;
    } else {
      return false;
    }
  }

  public void setHasAlgaeOverride(boolean passedHasAlgae) {
    hasAlgaeOverride = passedHasAlgae;
  }

  public void algaeToggle() {
    this.hasAlgaeOverride = !hasAlgaeOverride;
  }

  public double getAlgaeIntakeVoltage() {
    return m_algaeIntake.getMotorVoltage().getValueAsDouble();
  }

  public void setAlgaeIntakeVoltage(double voltage) {
    m_algaeIntake.setVoltage(voltage);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Algae/hasAlgaeOverride", hasAlgaeOverride);
    SmartDashboard.putBoolean("Algae/stateRun", stateRun);

    if (hasAlgae() && !stateRun) {
      setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_HOLD_SPEED);
    } else if (!stateRun) {
      setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_IDLE_SPEED);
    }
  }
}