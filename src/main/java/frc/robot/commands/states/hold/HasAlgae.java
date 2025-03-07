// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.hold;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS.CONSTANTS_ALGAE;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_WRIST;
import frc.robot.CONSTANTS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.State;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.State.RobotState;

public class HasAlgae extends Command {
    State state;
    Wrist wrist;
    Elevator elevator;
    Algae algae;
    Coral coral;

    public HasAlgae(RobotContainer RC) {
        this.state = RC.getState();
        this.wrist = RC.getWrist();
        this.elevator = RC.getElevator();
        this.algae = RC.getAlgae();
        this.coral = RC.getCoral();
        addRequirements(state);
    }

    @Override
    public void initialize() {
        state.setRobotState(RobotState.HAS_ALGAE);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        algae.setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_HOLD_SPEED);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
