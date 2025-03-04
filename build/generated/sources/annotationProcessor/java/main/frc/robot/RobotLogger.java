package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class RobotLogger extends ClassSpecificLogger<Robot> {
  private static final VarHandle $m_autonomousCommand;
  private static final VarHandle $bothSubsystemsZeroed;
  private static final VarHandle $m_robotContainer;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(Robot.class, MethodHandles.lookup());
      $m_autonomousCommand = lookup.findVarHandle(Robot.class, "m_autonomousCommand",
          edu.wpi.first.wpilibj2.command.Command.class);
      $bothSubsystemsZeroed = lookup.findVarHandle(Robot.class, "bothSubsystemsZeroed", boolean.class);
      $m_robotContainer = lookup.findVarHandle(Robot.class, "m_robotContainer", frc.robot.RobotContainer.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public RobotLogger() {
    super(Robot.class);
  }

  @Override
  public void update(EpilogueBackend backend, Robot object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("hasAutonomousRun", object.hasAutonomousRun);
      backend.log("bothSubsystemsZeroed", ((boolean) $bothSubsystemsZeroed.get(object)));
      Epilogue.robotContainerLogger.tryUpdate(backend.getNested("m_robotContainer"),
          ((frc.robot.RobotContainer) $m_robotContainer.get(object)), Epilogue.getConfig().errorHandler);
      backend.log("getMatchTime", object.getMatchTime());
    }
  }
}
