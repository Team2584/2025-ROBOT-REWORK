package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_VISION;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Drivetrain;

public class AddVisionMeasurement extends Command {
    Drivetrain drivetrain;
    Vision vision;

    PoseEstimate estimatedPose;
    double drivetrainRotation = 0;
    // NetworkTableInstance nt = NetworkTableInstance.getDefault();
    // StructPublisher<Pose2d> posePublisher =
    // nt.getTable(CONSTANTS_VISION.LIMELIGHT_NAMES[0])
    // .getStructTopic("pose2d", Pose2d.struct).publish();

    public AddVisionMeasurement(RobotContainer RC) {
        this.drivetrain = RC.getDrivetrain();
        this.vision = RC.getVision();

        addRequirements(vision);

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
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

        LimelightHelpers.SetIMUMode(CONSTANTS_VISION.LIMELIGHT_NAMES[0], 0);

        var estimatedPose = vision.determinePoseEstimate(gyroRate);
        if (estimatedPose.isPresent()) {
            // posePublisher.set(estimatedPose.get().pose);
            drivetrain.addVisionMeasurement(estimatedPose.get().pose, estimatedPose.get().timestampSeconds);
            if (DriverStation.isDisabled()) {
                drivetrain.resetYaw(estimatedPose.get().pose.getRotation().getDegrees());
            }
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
