package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class AlgaeLogger extends ClassSpecificLogger<Algae> {
  private static final VarHandle $m_algaeIntake;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(Algae.class, MethodHandles.lookup());
      $m_algaeIntake = lookup.findVarHandle(Algae.class, "m_algaeIntake", com.ctre.phoenix6.hardware.TalonFX.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public AlgaeLogger() {
    super(Algae.class);
  }

  @Override
  public void update(EpilogueBackend backend, Algae object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      logSendable(backend.getNested("m_algaeIntake"), ((com.ctre.phoenix6.hardware.TalonFX) $m_algaeIntake.get(object)));
      backend.log("hasAlgae", object.hasAlgae());
      backend.log("getAlgaeIntakeVoltage", object.getAlgaeIntakeVoltage());
    }
  }
}
