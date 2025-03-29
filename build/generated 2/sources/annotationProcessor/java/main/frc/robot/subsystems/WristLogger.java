package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class WristLogger extends ClassSpecificLogger<Wrist> {
  private static final VarHandle $m_wrist;
  private static final VarHandle $lastDesiredAngle;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(Wrist.class, MethodHandles.lookup());
      $m_wrist = lookup.findVarHandle(Wrist.class, "m_wrist", com.ctre.phoenix6.hardware.TalonFX.class);
      $lastDesiredAngle = lookup.findVarHandle(Wrist.class, "lastDesiredAngle", edu.wpi.first.units.measure.Angle.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public WristLogger() {
    super(Wrist.class);
  }

  @Override
  public void update(EpilogueBackend backend, Wrist object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      logSendable(backend.getNested("m_wrist"), ((com.ctre.phoenix6.hardware.TalonFX) $m_wrist.get(object)));
      backend.log("lastDesiredAngle", ((edu.wpi.first.units.measure.Angle) $lastDesiredAngle.get(object)));
      backend.log("getMotorVelocity", object.getMotorVelocity());
      backend.log("isMotorVelocityZero", object.isMotorVelocityZero());
      backend.log("getPivotAngle", object.getPivotAngle());
      backend.log("isAtSetPoint", object.isAtSetPoint());
      backend.log("isAtAnyAlgaeScoringPosition", object.isAtAnyAlgaeScoringPosition());
    }
  }
}
