package frc.robot.subsystems;

import javax.print.attribute.standard.Destination;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ALGAE;
import frc.robot.CONSTANTS.CONSTANTS_DRIVETRAIN;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.subsystems.swerve.Drivetrain;

@Logged
public class State extends SubsystemBase {
  public static DriverState currentDriverState;

  @NotLogged
  RobotContainer RC;

  // TODO: ADD NEW SUBSYSTEMS WHEN CREATED!!!!
  public State(RobotContainer RC) {

    currentDriverState = DriverState.MANUAL;

    this.RC = RC;
  }

  public void setDriverState(DriverState driverState) {
    currentDriverState = driverState;
  }

  public DriverState getDriverState() {
    return currentDriverState;
  }

  public static enum DriverState {
    MANUAL,

    REEF_ROTATION_SNAPPING,
    CORAL_STATION_ROTATION_SNAPPING,
    PROCESSOR_ROTATION_SNAPPING,

    REEF_AUTO_DRIVING,
    BARGE_AUTO_DRIVING,
    CORAL_STATION_AUTO_DRIVING,
    PROCESSOR_AUTO_DRIVING,
  }

  @Override
  public void periodic() {

  }
}