package frc.robot.subsystems.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

public class DrivetrainLogger extends ClassSpecificLogger<Drivetrain> {
  public DrivetrainLogger() {
    super(Drivetrain.class);
  }

  @Override
  public void update(EpilogueBackend backend, Drivetrain object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("desiredAlignmentPose", object.desiredAlignmentPose, edu.wpi.first.math.geometry.Pose2d.struct);
      backend.log("desiredModuleStates", object.desiredModuleStates, edu.wpi.first.math.kinematics.SwerveModuleState.struct);
      backend.log("actualModuleStates", object.actualModuleStates, edu.wpi.first.math.kinematics.SwerveModuleState.struct);
      backend.log("getRotationMeasure", object.getRotationMeasure());
      backend.log("getDesiredProcessor", object.getDesiredProcessor(), edu.wpi.first.math.geometry.Pose2d.struct);
    }
  }
}
