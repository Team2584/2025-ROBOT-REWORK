package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class ElevatorLogger extends ClassSpecificLogger<Elevator> {
  private static final VarHandle $m_Follower_Left;
  private static final VarHandle $m_Leader_Right;
  private static final VarHandle $elevatorZeroLimit;
  private static final VarHandle $lastDesiredPosition;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(Elevator.class, MethodHandles.lookup());
      $m_Follower_Left = lookup.findVarHandle(Elevator.class, "m_Follower_Left", com.ctre.phoenix6.hardware.TalonFX.class);
      $m_Leader_Right = lookup.findVarHandle(Elevator.class, "m_Leader_Right", com.ctre.phoenix6.hardware.TalonFX.class);
      $elevatorZeroLimit = lookup.findVarHandle(Elevator.class, "elevatorZeroLimit", edu.wpi.first.wpilibj.DigitalInput.class);
      $lastDesiredPosition = lookup.findVarHandle(Elevator.class, "lastDesiredPosition", edu.wpi.first.units.measure.Distance.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public ElevatorLogger() {
    super(Elevator.class);
  }

  @Override
  public void update(EpilogueBackend backend, Elevator object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      logSendable(backend.getNested("m_Follower_Left"), ((com.ctre.phoenix6.hardware.TalonFX) $m_Follower_Left.get(object)));
      logSendable(backend.getNested("m_Leader_Right"), ((com.ctre.phoenix6.hardware.TalonFX) $m_Leader_Right.get(object)));
      logSendable(backend.getNested("elevatorZeroLimit"), ((edu.wpi.first.wpilibj.DigitalInput) $elevatorZeroLimit.get(object)));
      backend.log("lastDesiredPosition", ((edu.wpi.first.units.measure.Distance) $lastDesiredPosition.get(object)));
      backend.log("currentLeftPosition", object.currentLeftPosition);
      backend.log("currentRightPosition", object.currentRightPosition);
      backend.log("tryingZero", object.tryingZero);
      backend.log("isZero", object.isZero);
      backend.log("getElevatorPosition", object.getElevatorPosition());
      backend.log("isAtSetPoint", object.isAtSetPoint());
      backend.log("isAtAnyCoralScoringPosition", object.isAtAnyCoralScoringPosition());
      backend.log("isAtAnyAlgaeScoringPosition", object.isAtAnyAlgaeScoringPosition());
      backend.log("getZeroLimit", object.getZeroLimit());
      backend.log("getMotorVelocity", object.getMotorVelocity());
      backend.log("isMotorVelocityZero", object.isMotorVelocityZero());
    }
  }
}
