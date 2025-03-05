package frc.robot.commands.states.action;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.State;
import frc.robot.subsystems.State.RobotState;

public class ScoreCoral extends SequentialCommandGroup {
    Coral globalCoral;
    Elevator globalElevator;
    State globalState;
    RobotState desiredState;
    double coralOuttakeSpeed;
  
    /** Creates a new CoralScoreSequence. */
    public ScoreCoral(RobotContainer RC, RobotState desiredState) {
      globalCoral = RC.getCoral();
      globalElevator = RC.getElevator();
      globalState = RC.getState();
      this.desiredState = desiredState;
  
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
          // Set state and LEDs
          Commands.runOnce(() -> globalState.setRobotState(RobotState.SCORE_CORAL)),
  
          // Shoot coral when elevator is at the right position
          //Commands.waitUntil(() -> globalElevator.isAtSetPoint()),
          Commands.runOnce(() -> globalCoral.setCoralMotor(getCoralOuttakeSpeed())),
  
          // Start ze timer
          Commands.waitSeconds(CONSTANTS_CORAL.CORAL_SCORE_TIME.in(Units.Seconds)),
  
          // Set the state to NONE once the timer is up and the operator lets go of the
          // button
          Commands.deferredProxy(() -> globalState.tryState(RobotState.NONE)));
    }
  
    public double getCoralOuttakeSpeed() {
      if (desiredState.equals(RobotState.PREP_CORAL_L4)) {
        coralOuttakeSpeed = CONSTANTS_CORAL.CORAL_L4_OUTTAKE_SPEED;
      } else if (desiredState.equals(RobotState.PREP_CORAL_L1)) {
        coralOuttakeSpeed = CONSTANTS_CORAL.CORAL_L1_OUTTAKE_SPEED;
      } else {
        coralOuttakeSpeed = CONSTANTS_CORAL.CORAL_OUTTAKE_SPEED;
      }
  
      return coralOuttakeSpeed;
    }
  }  
