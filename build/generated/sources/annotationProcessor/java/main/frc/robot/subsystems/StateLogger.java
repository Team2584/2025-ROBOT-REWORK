package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

public class StateLogger extends ClassSpecificLogger<State> {
  public StateLogger() {
    super(State.class);
  }

  @Override
  public void update(EpilogueBackend backend, State object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("getDriverState", object.getDriverState());
      backend.log("getRobotState", object.getRobotState());
      Epilogue.drivetrainLogger.tryUpdate(backend.getNested("getDrivetrain"), object.getDrivetrain(), Epilogue.getConfig().errorHandler);
      Epilogue.elevatorLogger.tryUpdate(backend.getNested("getElevator"), object.getElevator(), Epilogue.getConfig().errorHandler);
    }
  }
}
