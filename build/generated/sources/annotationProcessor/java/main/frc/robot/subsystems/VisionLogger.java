package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class VisionLogger extends ClassSpecificLogger<Vision> {
  private static final VarHandle $useMegaTag2;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(Vision.class, MethodHandles.lookup());
      $useMegaTag2 = lookup.findVarHandle(Vision.class, "useMegaTag2", boolean.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public VisionLogger() {
    super(Vision.class);
  }

  @Override
  public void update(EpilogueBackend backend, Vision object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("rightPose", object.rightPose, edu.wpi.first.math.geometry.Pose2d.struct);
      backend.log("leftPose", object.leftPose, edu.wpi.first.math.geometry.Pose2d.struct);
      backend.log("useMegaTag2", ((boolean) $useMegaTag2.get(object)));
    }
  }
}
