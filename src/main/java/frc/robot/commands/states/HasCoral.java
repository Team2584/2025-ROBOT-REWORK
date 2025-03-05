package frc.robot.commands.states;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.State.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HasCoral extends Command {
  /** Creates a new HasCoral. */
  State globalState;
  Coral globalCoral;
  Elevator globalElevator;

  public HasCoral(RobotContainer RC) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalState = RC.getState();
    globalCoral = RC.getCoral();
    globalElevator = RC.getElevator();
    addRequirements(globalState);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalState.setRobotState(RobotState.HAS_CORAL);
    globalElevator.setPosition(Units.Inches.zero());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}