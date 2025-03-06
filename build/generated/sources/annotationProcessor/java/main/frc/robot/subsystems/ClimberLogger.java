package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class ClimberLogger extends ClassSpecificLogger<Climber> {
  private static final VarHandle $m_climb;
  private static final VarHandle $lastTargetPosition;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(Climber.class, MethodHandles.lookup());
      $m_climb = lookup.findVarHandle(Climber.class, "m_climb", com.ctre.phoenix6.hardware.TalonFX.class);
      $lastTargetPosition = lookup.findVarHandle(Climber.class, "lastTargetPosition", edu.wpi.first.units.measure.Angle.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public ClimberLogger() {
    super(Climber.class);
  }

  @Override
  public void update(EpilogueBackend backend, Climber object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      logSendable(backend.getNested("m_climb"), ((com.ctre.phoenix6.hardware.TalonFX) $m_climb.get(object)));
      backend.log("lastTargetPosition", ((edu.wpi.first.units.measure.Angle) $lastTargetPosition.get(object)));
      backend.log("getClimberPosition", object.getClimberPosition());
      backend.log("isClimbDeployed", object.isClimbDeployed());
      backend.log("isClimbRetracted", object.isClimbRetracted());
    }
  }
}
