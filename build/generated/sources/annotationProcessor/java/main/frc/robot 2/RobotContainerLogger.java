package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class RobotContainerLogger extends ClassSpecificLogger<RobotContainer> {
  private static final VarHandle $state;
  private static final VarHandle $drivetrain;
  private static final VarHandle $elevator;
  private static final VarHandle $climber;
  private static final VarHandle $ramp;
  private static final VarHandle $wrist;
  private static final VarHandle $algae;
  private static final VarHandle $coral;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(RobotContainer.class, MethodHandles.lookup());
      $state = lookup.findVarHandle(RobotContainer.class, "state", frc.robot.subsystems.State.class);
      $drivetrain = lookup.findVarHandle(RobotContainer.class, "drivetrain", frc.robot.subsystems.swerve.Drivetrain.class);
      $elevator = lookup.findVarHandle(RobotContainer.class, "elevator", frc.robot.subsystems.Elevator.class);
      $climber = lookup.findVarHandle(RobotContainer.class, "climber", frc.robot.subsystems.Climber.class);
      $ramp = lookup.findVarHandle(RobotContainer.class, "ramp", frc.robot.subsystems.Ramp.class);
      $wrist = lookup.findVarHandle(RobotContainer.class, "wrist", frc.robot.subsystems.Wrist.class);
      $algae = lookup.findVarHandle(RobotContainer.class, "algae", frc.robot.subsystems.Algae.class);
      $coral = lookup.findVarHandle(RobotContainer.class, "coral", frc.robot.subsystems.Coral.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public RobotContainerLogger() {
    super(RobotContainer.class);
  }

  @Override
  public void update(EpilogueBackend backend, RobotContainer object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      Epilogue.stateLogger.tryUpdate(backend.getNested("state"), ((frc.robot.subsystems.State) $state.get(object)), Epilogue.getConfig().errorHandler);
      Epilogue.drivetrainLogger.tryUpdate(backend.getNested("drivetrain"), ((frc.robot.subsystems.swerve.Drivetrain) $drivetrain.get(object)), Epilogue.getConfig().errorHandler);
      Epilogue.elevatorLogger.tryUpdate(backend.getNested("elevator"), ((frc.robot.subsystems.Elevator) $elevator.get(object)), Epilogue.getConfig().errorHandler);
      Epilogue.climberLogger.tryUpdate(backend.getNested("climber"), ((frc.robot.subsystems.Climber) $climber.get(object)), Epilogue.getConfig().errorHandler);
      Epilogue.rampLogger.tryUpdate(backend.getNested("ramp"), ((frc.robot.subsystems.Ramp) $ramp.get(object)), Epilogue.getConfig().errorHandler);
      Epilogue.wristLogger.tryUpdate(backend.getNested("wrist"), ((frc.robot.subsystems.Wrist) $wrist.get(object)), Epilogue.getConfig().errorHandler);
      Epilogue.algaeLogger.tryUpdate(backend.getNested("algae"), ((frc.robot.subsystems.Algae) $algae.get(object)), Epilogue.getConfig().errorHandler);
      backend.log("SELECTED_AUTO_PREP_MAP_NAME", object.SELECTED_AUTO_PREP_MAP_NAME);
      backend.log("AUTO_PREP_NUM", object.AUTO_PREP_NUM);
      backend.log("slowModeTrigger", object.slowModeTrigger.getAsBoolean());
      backend.log("leftReefTrigger", object.leftReefTrigger.getAsBoolean());
      backend.log("rightReefTrigger", object.rightReefTrigger.getAsBoolean());
      backend.log("rightCoralStationTrigger", object.rightCoralStationTrigger.getAsBoolean());
      backend.log("leftCoralStationTrigger", object.leftCoralStationTrigger.getAsBoolean());
      backend.log("processorTrigger", object.processorTrigger.getAsBoolean());
      Epilogue.stateLogger.tryUpdate(backend.getNested("getState"), object.getState(), Epilogue.getConfig().errorHandler);
      Epilogue.drivetrainLogger.tryUpdate(backend.getNested("getDrivetrain"), object.getDrivetrain(), Epilogue.getConfig().errorHandler);
      Epilogue.elevatorLogger.tryUpdate(backend.getNested("getElevator"), object.getElevator(), Epilogue.getConfig().errorHandler);
      Epilogue.climberLogger.tryUpdate(backend.getNested("getClimber"), object.getClimber(), Epilogue.getConfig().errorHandler);
      Epilogue.rampLogger.tryUpdate(backend.getNested("getRamp"), object.getRamp(), Epilogue.getConfig().errorHandler);
      Epilogue.wristLogger.tryUpdate(backend.getNested("getWrist"), object.getWrist(), Epilogue.getConfig().errorHandler);
      Epilogue.algaeLogger.tryUpdate(backend.getNested("getAlgae"), object.getAlgae(), Epilogue.getConfig().errorHandler);
    }
  }
}
