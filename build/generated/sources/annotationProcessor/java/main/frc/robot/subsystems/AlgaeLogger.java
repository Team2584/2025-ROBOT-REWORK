package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

public class AlgaeLogger extends ClassSpecificLogger<Algae> {
  public AlgaeLogger() {
    super(Algae.class);
  }

  @Override
  public void update(EpilogueBackend backend, Algae object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      logSendable(backend.getNested("m_algaeIntake"), object.m_algaeIntake);
      backend.log("hasAlgaeOverride", object.hasAlgaeOverride);
      backend.log("hasAlgae", object.hasAlgae());
      backend.log("getAlgaeIntakeVoltage", object.getAlgaeIntakeVoltage());
    }
  }
}
