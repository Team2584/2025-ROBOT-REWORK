package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class RampLogger extends ClassSpecificLogger<Ramp> {
  private static final VarHandle $lastTargetPosition;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(Ramp.class, MethodHandles.lookup());
      $lastTargetPosition = lookup.findVarHandle(Ramp.class, "lastTargetPosition", edu.wpi.first.units.measure.Angle.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public RampLogger() {
    super(Ramp.class);
  }

  @Override
  public void update(EpilogueBackend backend, Ramp object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      logSendable(backend.getNested("m_ramp"), object.m_ramp);
      backend.log("lastTargetPosition", ((edu.wpi.first.units.measure.Angle) $lastTargetPosition.get(object)));
      backend.log("getRampPosition", object.getRampPosition());
      backend.log("isRampDown", object.isRampDown());
      backend.log("isRampUp", object.isRampUp());
    }
  }
}
