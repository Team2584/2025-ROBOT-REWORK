package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CONSTANTS.*;
import frc.robot.commands.NeutralState;
import frc.robot.commands.NeutralStateHandler;
import frc.robot.commands.TOFDrive;
import frc.robot.commands.prep_algae.PickupReefHighAlgae;
import frc.robot.commands.prep_algae.PickupReefLowAlgae;
import frc.robot.commands.prep_algae.PrepNetAlgae;
import frc.robot.commands.prep_coral.PrepCoralLock;
import frc.robot.commands.prep_coral.PrepCoralLvl4;
import frc.robot.subsystems.State.DriverState;

public class Autons {
    private static SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final RobotContainer RC;

    public Autons(RobotContainer RC) {
        this.RC = RC;
        configureAutoSelector();
        configureAutoBindings();
    }

    // TODO: register commands
    public static void configurePPCommands() {
        NamedCommands.registerCommand("X", new InstantCommand());
    }

    public static Command L4FourPieceHigh(RobotContainer RC) {
        EventTrigger neutral = new EventTrigger("neutral");
        neutral.onTrue(new NeutralStateHandler(RC));

        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("P-J"),
                driveAutoAlign(RC, 9, 0.6),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC),
                new WaitCommand(0.2),

                EnsureNeutralState(RC),
                RC.getDrivetrain().runPathT("J-TOP"),
                GetCoralStationPiece(RC),
                RC.getDrivetrain().runPathT("TOP-K"),
                driveAutoAlign(RC, 10, 0.6),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC),
                new WaitCommand(0.2),

                EnsureNeutralState(RC),
                RC.getDrivetrain().runPathT("K-TOP"),
                GetCoralStationPiece(RC),
                RC.getDrivetrain().runPathT("TOP-L"),
                driveAutoAlign(RC, 11, 0.6),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC),
                new WaitCommand(0.2),

                EnsureNeutralState(RC),
                RC.getDrivetrain().runPathT("L-TOP"),
                GetCoralStationPiece(RC),
                RC.getDrivetrain().runPathT("TOP-A"),
                driveAutoAlign(RC, 0, 0.6),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC));
    }

    public static Command L4FourPieceLow(RobotContainer RC) {
        EventTrigger neutral = new EventTrigger("neutral");
        neutral.onTrue(new NeutralStateHandler(RC));

        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("P-E"),
                driveAutoAlign(RC, 4, 0.6),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC),
                new WaitCommand(0.2),

                EnsureNeutralState(RC),
                RC.getDrivetrain().runPathT("E-TOP"),
                GetCoralStationPiece(RC),
                RC.getDrivetrain().runPathT("TOP-D"),
                driveAutoAlign(RC, 3, 0.6),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC),
                new WaitCommand(0.2),

                EnsureNeutralState(RC),
                RC.getDrivetrain().runPathT("D-TOP"),
                GetCoralStationPiece(RC),
                RC.getDrivetrain().runPathT("TOP-C"),
                driveAutoAlign(RC, 2, 0.6),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC),
                new WaitCommand(0.2),

                EnsureNeutralState(RC),
                RC.getDrivetrain().runPathT("C-TOP"),
                GetCoralStationPiece(RC),
                RC.getDrivetrain().runPathT("TOP-B"),
                driveAutoAlign(RC, 1, 0.6),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC));
    }

    public static Command CoralStationTest(RobotContainer RC) {
        return new SequentialCommandGroup(
                GetCoralStationPiece(RC),
                GoL4(RC));
    }

    /*
     * Scores L4 H Coral
     * Picks up H Algae and scores into barge
     * Resets pos
     * TODO: Set Algae mech to be able to intake algae, drive in, drive out
     */
    public static Command L4CenterAlgae(RobotContainer RC) {
        EventTrigger pickupLowAlgae = new EventTrigger("pickupLowAlgae");
        pickupLowAlgae.onTrue(new PickupReefLowAlgae(RC).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT));

        EventTrigger pickupHighAlgae = new EventTrigger("pickupHighAlgae");
        pickupHighAlgae.onTrue(new PickupReefHighAlgae(RC).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT));

        EventTrigger NeutralState = new EventTrigger("NeutralState");
        NeutralState.onTrue(new NeutralStateHandler(RC));

        EventTrigger PepareNetAlgae = new EventTrigger("PepareNetAlgae");
        PepareNetAlgae.onTrue(new PrepNetAlgae(RC));

        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("P-H"),
                driveAutoAlign(RC, 7, 1),
                GoL4(RC),
                new WaitCommand(0.75),
                TOFDriveScore(RC),
                new WaitCommand(0.5),
                RC.getDrivetrain().runPathT("CoralToAlgae"),
                new WaitCommand(0.2),
                SetAlgaeLow(RC),
                new WaitCommand(0.5),
                RC.getDrivetrain().runPathT("RetrieveAlgae"),
                RC.getDrivetrain().runPathT("BackupAlgae"),
                RC.getDrivetrain().runPathT("ScoreMidAlgae"),
                new WaitCommand(0.4),
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_OUTTAKE_SPEED)),
                new WaitCommand(0.5),
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(0)),
                RC.getDrivetrain().runPathT("CenterBargeTop"));
    }

    /*
     * Scores L4 H Coral
     * Picks up H Algae and scores into barge
     * Resets pos
     * TODO: Set Algae mech to be able to intake algae, drive in, drive out
     */
    public static Command L4CenterAlgaeTickle(RobotContainer RC) {
        EventTrigger pickupLowAlgae = new EventTrigger("pickupLowAlgae");
        pickupLowAlgae.onTrue(new PickupReefLowAlgae(RC).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT));

        EventTrigger pickupHighAlgae = new EventTrigger("pickupHighAlgae");
        pickupHighAlgae.onTrue(new PickupReefHighAlgae(RC).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT));

        EventTrigger NeutralState = new EventTrigger("NeutralState");
        NeutralState.onTrue(new NeutralStateHandler(RC));

        EventTrigger PepareNetAlgae = new EventTrigger("PepareNetAlgae");
        PepareNetAlgae.onTrue(new PrepNetAlgae(RC));

        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("CenterTickle"),
                RC.getDrivetrain().runPathT("P-H"),
                driveAutoAlign(RC, 7, 1),
                GoL4(RC),
                new WaitCommand(0.75),
                TOFDriveScore(RC),
                new WaitCommand(0.5),
                RC.getDrivetrain().runPathT("CoralToAlgae"),
                new WaitCommand(0.2),
                SetAlgaeLow(RC),
                new WaitCommand(0.5),
                RC.getDrivetrain().runPathT("RetrieveAlgae"),
                RC.getDrivetrain().runPathT("BackupAlgae"),
                EnsureNeutralStateHandler(RC) //

        // RC.getDrivetrain().runPathT("ScoreMidAlgae"),
        // new WaitCommand(0.4),
        // new InstantCommand(() ->
        // RC.getAlgae().setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_OUTTAKE_SPEED)),
        // new WaitCommand(0.5),
        // new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(0)),
        // RC.getDrivetrain().runPathT("Barge-Algae-Reset")
        // SetAlgaeHigh(RC),
        // RC.getDrivetrain().runPathT("I-Algae-Backup")
        );
    }

    /*
     * Scores L4 E Coral
     * Picks up E Algae and holds
     */
    public static Command L4OnePieceLow(RobotContainer RC) {
        EventTrigger NeutralState = new EventTrigger("NeutralState");
        NeutralState.onTrue(new NeutralStateHandler(RC));

        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("P-E"),
                driveAutoAlign(RC, 4, 1),
                GoL4(RC),
                new WaitCommand(0.75),
                TOFDriveScore(RC),
                new WaitCommand(0.5),
                RC.getDrivetrain().runPathT("E-CoralToAlgaeSetup"),
                new WaitCommand(0.2),
                SetAlgaeHigh(RC),
                new WaitCommand(1),
                RC.getDrivetrain().runPathT("E-AlgaeIntake"),
                RC.getDrivetrain().runPathT("E-SafeAlgaeBackup"),
                EnsureNeutralStateHandler(RC));
    }

    /*
     * Scores L4 J Coral
     * Picks up J Algae and scores in barge
     */
    public static Command L4OnePieceHigh(RobotContainer RC) {
        EventTrigger NeutralState = new EventTrigger("NeutralState");
        NeutralState.onTrue(new NeutralStateHandler(RC));

        EventTrigger PepareNetAlgae = new EventTrigger("PepareNetAlgae");
        PepareNetAlgae.onTrue(new PrepNetAlgae(RC));

        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("P-J"),
                driveAutoAlign(RC, 9, 1),
                GoL4(RC),
                new WaitCommand(0.75),
                TOFDriveScore(RC),
                new WaitCommand(0.5),
                RC.getDrivetrain().runPathT("J-CoralToAlgaeSetup"),
                new WaitCommand(0.2),
                SetAlgaeHigh(RC),
                new WaitCommand(1),
                RC.getDrivetrain().runPathT("J-AlgaeIntake"),
                RC.getDrivetrain().runPathT("J-AlgaeBackup"),
                RC.getDrivetrain().runPathT("SingleAlgaeHighBarge"),
                new WaitCommand(0.4),
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_OUTTAKE_SPEED)),
                new WaitCommand(0.5),
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(0)),
                RC.getDrivetrain().runPathT("SingleAlgaeHighBargeSafe"));
    }

    // ---** COMMANDS **---

    public static Command driveAutoAlign(RobotContainer RC, int reefIndex) {
        return Commands.runOnce(() -> RC.getDrivetrain().autoAlign(Meters.of(0),
                CONSTANTS_FIELD.getReefPositions().get().get(reefIndex), MetersPerSecond.of(0),
                MetersPerSecond.of(0), DegreesPerSecond.of(0), 1.0, true, Meters.of(1000),
                DriverState.REEF_AUTO_DRIVING,
                DriverState.REEF_AUTO_DRIVING, RC.getState())).repeatedly();
    }

    public static Command driveAutoAlign(RobotContainer RC, int reefIndex, double timeOut) {
        return driveAutoAlign(RC, reefIndex).asProxy().withTimeout(timeOut);
    }

    public static Command PlaceL4Sequence(RobotContainer RC, int reefIndex, double alignTimeout) {
        return Commands.sequence(
                driveAutoAlign(RC, reefIndex, alignTimeout),
                new PrepCoralLvl4(RC).asProxy().withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT),
                new TOFDrive(RC, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE_AUTO)
                        .andThen(RC.getCoral().outtakeCoral().withTimeout(0.125)));
    }

    public static Command GetCoralStationPiece(RobotContainer RC) {
        return new SequentialCommandGroup(new ParallelCommandGroup(
            new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.ZEROED_POS)),
            RC.getCoral().intakeCoral(),
            new InstantCommand(() -> RC.getRamp().setRampMotorVelocity(CONSTANTS_RAMP.RAMP_INTAKE_VELOCITY)),
            RC.getWrist().setWristAngleCommand(CONSTANTS_WRIST.PIVOT_INTAKE_CORAL)).until(() -> RC.getCoral().coralLoaded())
            .andThen(new InstantCommand(() -> RC.getRamp().setRampMotorVelocity(0))),

            new InstantCommand(() -> new PrepCoralLock(RC).schedule()));
    }

    public static Command GoL4(RobotContainer RC) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4))
                        .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT),

                new InstantCommand(() -> RC.getWrist().setWristAngle(CONSTANTS_WRIST.PIVOT_SCORE_CORAL)));
    }

    public static Command SetAlgaeLow(RobotContainer RC) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> RC.getWrist().setWristAngle(CONSTANTS_WRIST.PIVOT_ALGAE_REEF))
                        .withTimeout(CONSTANTS_WRIST.WRIST_TIMEOUT),
                new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_ALGAE_LOW)),
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_INTAKE_SPEED)));
    }

    public static Command SetAlgaeHigh(RobotContainer RC) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> RC.getWrist().setWristAngle(CONSTANTS_WRIST.PIVOT_ALGAE_REEF))
                        .withTimeout(CONSTANTS_WRIST.WRIST_TIMEOUT),
                new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_ALGAE_HIGH)),
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_INTAKE_SPEED)));
    }

    private static Command TOFDriveScore(RobotContainer RC) {
        return new TOFDrive(RC, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE_AUTO)
                .andThen(RC.getCoral().outtakeCoral().withTimeout(0.125));
    }

    public static Command EnsureNeutralStateHandler(RobotContainer RC) {
        return new NeutralStateHandler(RC);
    }

    public static Command EnsureNeutralState(RobotContainer RC) {
        return new NeutralState(RC);
    }

    private void configureAutoBindings() {

        autoChooser.addOption("L4FourPieceHigh", L4FourPieceHigh(RC));
        autoChooser.addOption("L4FourPieceLow", L4FourPieceLow(RC));
        autoChooser.addOption("L4CenterAlgae", L4CenterAlgae(RC));
        // autoChooser.addOption("L4CenterAlgaeTickle", L4CenterAlgaeTickle(RC));
        autoChooser.addOption("L4OnePieceLow", L4OnePieceLow(RC));
        autoChooser.addOption("L4OnePieceHigh", L4OnePieceHigh(RC));
        autoChooser.addOption("CoralStationTest", CoralStationTest(RC));

        // autoChooser.setDefaultOption("L4_4_HIGH", L4FourPieceHigh(RC));
    }

    public static void runAuton(String auto) { // Autons.runAuton(auto);

    }

    // ---** AUTON INSTANTIATING STUFF **---

    public static Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureAutoSelector() {
        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData(autoChooser);
    }
}