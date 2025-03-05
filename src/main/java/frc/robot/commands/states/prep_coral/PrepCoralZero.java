package frc.robot.commands.states.prep_coral;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.subsystems.*;
import frc.robot.subsystems.State.RobotState;

public class PrepCoralZero extends Command {
    State globalState;
    Elevator globalElevator;

    public PrepCoralZero(RobotContainer RC) {
        globalState = RC.getState();
        globalElevator = RC.getElevator();
        addRequirements(globalState);
        addRequirements(globalElevator);
    }

    @Override
    public void initialize() {
      globalState.setRobotState(RobotState.PREP_CORAL_ZERO);

      globalElevator.setPosition(Units.Inches.of(0));
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Elevator height: ", globalElevator.getElevatorPosition().in(Inches));
    }
  
    @Override
    public void end(boolean interrupted) {
    }
  
    @Override
    public boolean isFinished() {
      return globalElevator.isAtSetPoint();
    }
}