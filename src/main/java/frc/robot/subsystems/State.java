package frc.robot.subsystems;

import javax.print.attribute.standard.Destination;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ALGAE;
import frc.robot.CONSTANTS.CONSTANTS_DRIVETRAIN;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.commands.states.None;
import frc.robot.commands.states.action.IntakeAlgae;
import frc.robot.commands.states.action.IntakeCoral;
import frc.robot.commands.states.action.ScoreAlgae;
import frc.robot.commands.states.action.ScoreCoral;
import frc.robot.commands.states.hold.HasCoral;
import frc.robot.commands.states.prep_algae.PrepAlgaeNet;
import frc.robot.commands.states.prep_algae.PrepIntakeAlgaeGround;
import frc.robot.commands.states.prep_algae.PrepIntakeAlgaeHighReef;
import frc.robot.commands.states.prep_algae.PrepIntakeAlgaeLowReef;
import frc.robot.commands.states.prep_coral.PrepCoralLvl;
import frc.robot.commands.states.prep_coral.PrepCoralLvl1;
import frc.robot.commands.states.prep_coral.PrepCoralLvl2;
import frc.robot.commands.states.prep_coral.PrepCoralLvl3;
import frc.robot.commands.states.prep_coral.PrepCoralLvl4;
import frc.robot.commands.states.prep_coral.PrepCoralZero;
import frc.robot.subsystems.swerve.Drivetrain;

@Logged
public class State extends SubsystemBase {
  public static DriverState currentDriverState;
  public static RobotState currentRobotState;

  @NotLogged
  RobotContainer RC;

  // TODO: ADD NEW SUBSYSTEMS WHEN CREATED!!!!
  public State(RobotContainer RC) {
    currentRobotState = RobotState.NONE;
    currentDriverState = DriverState.MANUAL;

    this.RC = RC;
  }

  public void setDriverState(DriverState driverState) {
    currentDriverState = driverState;
  }

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public DriverState getDriverState() {
    return currentDriverState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  public Command tryState(RobotState desiredState) {
    switch (desiredState) {

      // --- Intermediary States ---

      case NONE:
        return new None(RC);

      case HAS_CORAL:
        switch (currentRobotState) {
          case INTAKE_CORAL:
          case SCORE_ALGAE_WITH_CORAL:
            return new HasCoral(RC);
        }
        break;

      case HAS_ALGAE:
        switch (desiredState) {
          case INTAKE_ALGAE:
          case SCORE_CORAL_WITH_ALGAE:
            // TODO
        }
        break;

      case HAS_CORAL_AND_ALGAE:
        switch (desiredState) {
          case INTAKE_CORAL_WITH_ALGAE:
          case INTAKE_ALGAE_GROUND_WITH_CORAL:
          case INTAKE_ALGAE_REEF_WITH_CORAL:
            // TODO
        }
        break;

      // --- Prep States ---

      // Prep Coral Only
      case PREP_CORAL_ZERO:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            return new PrepCoralZero(RC);
        }
        break;

      case PREP_CORAL_L1:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_ZERO:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            return new PrepCoralLvl1(RC);
        }
        break;

      case PREP_CORAL_L2:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_ZERO:
          case PREP_CORAL_L1:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            return new PrepCoralLvl2(RC);
        }
        break;

      case PREP_CORAL_L3:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_ZERO:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L4:
            return new PrepCoralLvl3(RC);
        }
        break;

      case PREP_CORAL_L4:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_ZERO:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
            return new PrepCoralLvl4(RC);
        }
        break;

      // Prep Coral w/Algae
      case PREP_CORAL_ZERO_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
            // TODO
        }
        break;

      case PREP_CORAL_L1_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
            // TODO
        }
        break;

      case PREP_CORAL_L2_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
            // TODO
        }
        break;

      case PREP_CORAL_L3_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
            // TODO
        }
        break;

      case PREP_CORAL_L4_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
            // TODO
        }
        break;

      // Prep Algae Only
      case PREP_ALGAE_INTAKE_GROUND:
        Commands.print("Ground");
        return new PrepIntakeAlgaeGround(RC);

      case PREP_ALGAE_INTAKE_REEF_LOW:
        Commands.print("Lowreef");
        return new PrepIntakeAlgaeLowReef(RC);

      case PREP_ALGAE_INTAKE_REEF_HIGH:
        Commands.print("Highreef");
        return new PrepIntakeAlgaeHighReef(RC);

      case PREP_ALGAE_BARGE:
        Commands.print("Barge");
        return new PrepAlgaeNet(RC);

      case PREP_ALGAE_PROCESSOR:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_BARGE:
            // TODO
        }
        break;

      // Prep Algae w/Coral
      case PREP_ALGAE_INTAKE_GROUND_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_ALGAE_INTAKE_LOW_WITH_CORAL:
          case PREP_ALGAE_INTAKE_HIGH_WITH_CORAL:
            // TODO
        }
        break;

      case PREP_ALGAE_INTAKE_LOW_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_ALGAE_INTAKE_GROUND_WITH_CORAL:
          case PREP_ALGAE_INTAKE_HIGH_WITH_CORAL:
            // TODO
        }
        break;

      case PREP_ALGAE_INTAKE_HIGH_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_ALGAE_INTAKE_GROUND_WITH_CORAL:
          case PREP_ALGAE_INTAKE_LOW_WITH_CORAL:
            // TODO
        }
        break;

      case PREP_ALGAE_ZERO_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_ALGAE_BARGE_WITH_CORAL:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
            // TODO
        }
        break;

      case PREP_ALGAE_BARGE_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_ALGAE_ZERO_WITH_CORAL:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
            // TODO
        }
        break;

      case PREP_ALGAE_PROCESSOR_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_ALGAE_ZERO_WITH_CORAL:
          case PREP_ALGAE_BARGE_WITH_CORAL:
            // TODO
        }
        break;

      // --- Action States ---

      // Actions Coral
      case INTAKE_CORAL:
        return new IntakeCoral(RC);

      case SCORE_CORAL:
        return new ScoreCoral(RC);

      case INTAKE_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_ALGAE:
            // TODO
        }
        break;

      case SCORE_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
            // TODO
        }
        break;

      // Actions Algae
      case INTAKE_ALGAE:
        return new IntakeAlgae(RC);

      // case INTAKE_ALGAE_REEF:
      // switch (currentRobotState) {
      // case NONE:
      // // TODO
      // }
      // break;

      case SCORE_ALGAE:
        return new ScoreAlgae(RC);

      case INTAKE_ALGAE_GROUND_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
            // TODO
        }
        break;

      case INTAKE_ALGAE_REEF_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
            // TODO
        }
        break;

      case SCORE_ALGAE_WITH_CORAL:
        switch (currentRobotState) {
          case PREP_ALGAE_ZERO_WITH_CORAL:
          case PREP_ALGAE_BARGE_WITH_CORAL:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
            // TODO
        }
        break;
    }

    return Commands
        .print("You gave it a no good state bud. You tried: " + desiredState.toString()
            + " while at " + currentRobotState.toString() + " and it broke...");
  }

  public static enum DriverState {
    MANUAL,

    REEF_ROTATION_SNAPPING,
    CORAL_STATION_ROTATION_SNAPPING,
    PROCESSOR_ROTATION_SNAPPING,

    REEF_AUTO_DRIVING,
    BARGE_AUTO_DRIVING,
    CORAL_STATION_AUTO_DRIVING,
    PROCESSOR_AUTO_DRIVING,
  }

  /**
   * Represents the various states the robot can be in
   */
  public static enum RobotState {

    //
    // --- INTERMEDIARY STATES ---
    //

    // Hold 0 Elements
    NONE,

    // Hold 1 Element
    HAS_CORAL,
    HAS_ALGAE,

    // Hold 2 Elements
    HAS_CORAL_AND_ALGAE,

    //
    // --- PREP STATES ---
    //

    // Prep Coral Only
    PREP_CORAL_ZERO,
    PREP_CORAL_L1,
    PREP_CORAL_L2,
    PREP_CORAL_L3,
    PREP_CORAL_L4,

    // Prep Coral w/Algae
    PREP_CORAL_ZERO_WITH_ALGAE,
    PREP_CORAL_L1_WITH_ALGAE,
    PREP_CORAL_L2_WITH_ALGAE,
    PREP_CORAL_L3_WITH_ALGAE,
    PREP_CORAL_L4_WITH_ALGAE,

    // Prep Algae Only
    PREP_ALGAE_INTAKE_GROUND,
    PREP_ALGAE_INTAKE_REEF_LOW,
    PREP_ALGAE_INTAKE_REEF_HIGH,
    PREP_ALGAE_BARGE,
    PREP_ALGAE_PROCESSOR,

    // Prep Algae w/Coral
    PREP_ALGAE_INTAKE_GROUND_WITH_CORAL,
    PREP_ALGAE_INTAKE_LOW_WITH_CORAL,
    PREP_ALGAE_INTAKE_HIGH_WITH_CORAL,

    PREP_ALGAE_ZERO_WITH_CORAL,
    PREP_ALGAE_BARGE_WITH_CORAL,
    PREP_ALGAE_PROCESSOR_WITH_CORAL,

    //
    // --- ACTION STATES ---
    //

    // Actions Coral
    INTAKE_CORAL,
    SCORE_CORAL,
    INTAKE_CORAL_WITH_ALGAE,
    SCORE_CORAL_WITH_ALGAE,

    // Actions Algae
    INTAKE_ALGAE,
    SCORE_ALGAE,
    INTAKE_ALGAE_GROUND_WITH_CORAL,
    INTAKE_ALGAE_REEF_WITH_CORAL,
    SCORE_ALGAE_WITH_CORAL,

    // Actions Climb
    CLIMBER_DEPLOYING,
    CLIMBER_RETRACTING
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Current Robot State", getRobotState().name());
  }
}