package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;

@Logged
public class State extends SubsystemBase {
  public static DriverState currentDriverState;

  @NotLogged
  RobotContainer RC;

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