package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_DRIVETRAIN;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_FIELD;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.State;
import frc.robot.subsystems.State.DriverState;
import frc.robot.subsystems.swerve.Drivetrain;

public class DriveTeleop extends Command {
  State state;
  Drivetrain drivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  BooleanSupplier slowMode, leftReef, rightReef, coralStationLeft, coralStationRight, processor;
  Elevator elevator;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;
  double slowMultiplier = 0;

  /**
   * @param subStateMachine
   * @param subDrivetrain
   * @param subElevator
   * @param xAxis
   * @param yAxis
   * @param rotationAxis
   * @param slowMode
   * @param leftReef
   * @param rightReef
   * @param coralStationLeft
   * @param coralStationRight
   * @param processorBtn
   */
  public DriveTeleop(RobotContainer RC, DoubleSupplier xAxis,
      DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, BooleanSupplier slowMode, BooleanSupplier leftReef, BooleanSupplier rightReef,
      BooleanSupplier coralStationLeft, BooleanSupplier coralStationRight,
      BooleanSupplier processorBtn) {
    this.state = RC.getState();
    this.drivetrain = RC.getDrivetrain();
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;
    this.leftReef = leftReef;
    this.rightReef = rightReef;
    this.coralStationLeft = coralStationLeft;
    this.coralStationRight = coralStationRight;
    this.elevator = RC.getElevator();
    this.processor = processorBtn;

    isOpenLoop = true;

    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    redAllianceMultiplier = CONSTANTS_FIELD.isRedAlliance() ? -1 : 1;
  }

  @Override
  public void execute() {
    // -- Multipliers --
    if (slowMode.getAsBoolean()) {
      slowMultiplier = CONSTANTS_DRIVETRAIN.SLOW_MODE_GOVERNOR;
    } else {
      slowMultiplier = 1;
    }

    // Get Joystick inputs
    double elevatorHeightMultiplier = SN_Math.interpolate(
        elevator.getElevatorPosition().in(Units.Meters),
        0.0, CONSTANTS_ELEVATOR.ELEVATOR_MAX_HEIGHT.in(Units.Meters),
        1.0, CONSTANTS_DRIVETRAIN.MINIMUM_ELEVATOR_GOVERNOR);

    double transMultiplier = slowMultiplier
        * CONSTANTS_DRIVETRAIN.MAX_DRIVE_SPEED.in(Units.MetersPerSecond) * elevatorHeightMultiplier;

    // -- Velocities --
    LinearVelocity xVelocity = Units.MetersPerSecond.of(xAxis.getAsDouble() * transMultiplier);
    LinearVelocity yVelocity = Units.MetersPerSecond.of(-yAxis.getAsDouble() * transMultiplier);
    AngularVelocity rVelocity = Units.RadiansPerSecond
        .of(-rotationAxis.getAsDouble() * CONSTANTS_DRIVETRAIN.TURN_SPEED.in(Units.RadiansPerSecond)
            * elevatorHeightMultiplier);

    // -- Controlling --
    if (leftReef.getAsBoolean() || rightReef.getAsBoolean()) {
      // Reef auto-align is requested
      Pose2d desiredReef = drivetrain.getDesiredReef(leftReef.getAsBoolean());
      Distance reefDistance = Units.Meters
          .of(drivetrain.getPose().getTranslation().getDistance(desiredReef.getTranslation()));

      // Begin reef auto align (rotationally, automatically driving, or w/ a driver
      // override)
      drivetrain.autoAlign(reefDistance, desiredReef, xVelocity, yVelocity, rVelocity, transMultiplier,
          isOpenLoop,
          CONSTANTS.CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_REEF_DISTANCE,
          DriverState.REEF_AUTO_DRIVING, DriverState.REEF_ROTATION_SNAPPING, state);

    }

    // -- Coral Station --
    else if (coralStationRight.getAsBoolean()) {
      Pose2d desiredCoralStation = CONSTANTS_FIELD.getCoralStationPositions().get().get(0);
      Distance coralStationDistance = Units.Meters
          .of(drivetrain.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));
      drivetrain.rotationalAutoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
          transMultiplier, isOpenLoop,
          CONSTANTS.CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
          DriverState.CORAL_STATION_AUTO_DRIVING, DriverState.CORAL_STATION_ROTATION_SNAPPING, state);
    }

    else if (coralStationLeft.getAsBoolean()) {
      Pose2d desiredCoralStation = CONSTANTS_FIELD.getCoralStationPositions().get().get(2);

      Distance coralStationDistance = Units.Meters
          .of(drivetrain.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));
      drivetrain.rotationalAutoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
          transMultiplier, isOpenLoop,
          CONSTANTS.CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
          DriverState.CORAL_STATION_AUTO_DRIVING, DriverState.CORAL_STATION_ROTATION_SNAPPING, state);
    }

    // -- Processors --
    else if (processor.getAsBoolean()) {
      Pose2d desiredProcessor = drivetrain.getDesiredProcessor();
      Distance processorDistance = Units.Meters
          .of(drivetrain.getPose().getTranslation().getDistance(desiredProcessor.getTranslation()));

      drivetrain.rotationalAutoAlign(processorDistance, desiredProcessor, xVelocity, yVelocity, rVelocity,
          transMultiplier,
          isOpenLoop, CONSTANTS.CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_PROCESSOR_DISTANCE,
          DriverState.PROCESSOR_AUTO_DRIVING, DriverState.PROCESSOR_ROTATION_SNAPPING, state);
    }

    else {
      // Regular driving
      drivetrain.drive(
          new Translation2d(xVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond),
              yVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond)),
          rVelocity.in(Units.RadiansPerSecond), isOpenLoop);
      state.setDriverState(DriverState.MANUAL);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}