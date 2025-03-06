// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.utils.LimelightHelpers;
import com.frcteam3255.utils.LimelightHelpers.PoseEstimate;
import java.util.Optional;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_VISION;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Drivetrain;

public class AddVisionMeasurement extends Command {
  Drivetrain drivetrain;
  Vision vision;

  PoseEstimate estimatedPose;
  double drivetrainRotation = 0;

  public AddVisionMeasurement(RobotContainer RC) {
    this.drivetrain = RC.getDrivetrain();
    this.vision = RC.getVision();

    addRequirements(vision);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Tells the limelight where we are on the field
    LimelightHelpers.SetRobotOrientation(CONSTANTS_VISION.LIMELIGHT_NAMES[0],
        drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(CONSTANTS_VISION.LIMELIGHT_NAMES[1],
        drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    AngularVelocity gyroRate = Units.DegreesPerSecond.of(drivetrain.getGyroRate());

    Optional<PoseEstimate> estimatedPose = vision.determinePoseEstimate(gyroRate);
    if (estimatedPose.isPresent()) {
      drivetrain.addVisionMeasurement(estimatedPose.get().pose, estimatedPose.get().timestampSeconds);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
