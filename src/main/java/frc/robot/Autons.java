package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CONSTANTS.*;
import frc.robot.commands.NeutralState;
import frc.robot.commands.NeutralStateHandler;
import frc.robot.commands.prep_coral.PrepCoralLock;
import frc.robot.commands.prep_coral.PrepIntakeCoralAuto;
import frc.robot.commands.prep_coral.PrepIntakeCoralAutoSecondary;
import frc.robot.subsystems.State.DriverState;

public class Autons {
    private static SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final RobotContainer RC;

    public Autons(RobotContainer RC) {
        this.RC = RC;
        configureAutoSelector();
        configureAutoBindings();
    }

    public static void configurePPCommands(RobotContainer RC) {
        // NamedCommands.registerCommand("X", new InstantCommand());
        // new EventTrigger("X").onTrue(new InstantCommand());
    }

    private static double INTAKE_MAX_WAIT_TIME = 3;

    public static Command L4FourPieceHigh(RobotContainer RC) {
        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("P-J"),
                new WaitCommand(0.15),
                driveAutoAlign(RC, 9, 1),
                GoL4Score(RC),

                EnsureNeutralState(RC),
                new WaitCommand(0.32),
                RC.getDrivetrain().runPathT("J-TOP"),
                GetCoralStationPiece(RC).withTimeout(INTAKE_MAX_WAIT_TIME),
                new ParallelDeadlineGroup(RC.getDrivetrain().runPathT("TOP-K"), GetCoralStationPieceSecondary(RC)),
                driveAutoAlign(RC, 10, 1),
                GoL4Score(RC),

                EnsureNeutralState(RC),
                new WaitCommand(0.32),
                RC.getDrivetrain().runPathT("K-TOP"),
                GetCoralStationPiece(RC).withTimeout(INTAKE_MAX_WAIT_TIME),
                new ParallelDeadlineGroup(RC.getDrivetrain().runPathT("TOP-L"), GetCoralStationPieceSecondary(RC)),
                driveAutoAlign(RC, 11, 0.5),
                GoL4Score(RC),

                EnsureNeutralState(RC),
                new WaitCommand(0.32),
                RC.getDrivetrain().runPathT("L-TOP"),
                GetCoralStationPiece(RC).withTimeout(INTAKE_MAX_WAIT_TIME),
                new ParallelDeadlineGroup(RC.getDrivetrain().runPathT("TOP-A"), GetCoralStationPieceSecondary(RC)),
                driveAutoAlign(RC, 0, 0.5),
                GoL4Score(RC));
    }

    public static Command L4FourPieceHighTICKLE(RobotContainer RC) {
        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("HighTickle"),
                RC.getDrivetrain().runPathT("P-J-Tickle"),
                driveAutoAlign(RC, 9, 0.7),
                GoL4Score(RC),

                EnsureNeutralState(RC),
                new WaitCommand(0.32),
                RC.getDrivetrain().runPathT("J-TOP"),
                GetCoralStationPiece(RC),
                RC.getDrivetrain().runPathT("TOP-K"),
                driveAutoAlign(RC, 10, 0.5),
                GoL4Score(RC),

                EnsureNeutralState(RC),
                new WaitCommand(0.32),
                RC.getDrivetrain().runPathT("K-TOP"),
                GetCoralStationPiece(RC),
                RC.getDrivetrain().runPathT("TOP-L"),
                driveAutoAlign(RC, 11, 0.5),
                GoL4Score(RC),

                EnsureNeutralState(RC),
                new WaitCommand(0.32),
                RC.getDrivetrain().runPathT("L-TOP"),
                GetCoralStationPiece(RC),
                RC.getDrivetrain().runPathT("TOP-A"),
                driveAutoAlign(RC, 0, 0.5),
                GoL4Score(RC));
    }

    public static Command L4FourPieceLow(RobotContainer RC) {
        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("P-E"),
                driveAutoAlign(RC, 4, 1),
                GoL4Score(RC),

                EnsureNeutralState(RC),
                new WaitCommand(0.32),
                RC.getDrivetrain().runPathT("E-TOP"),
                GetCoralStationPiece(RC).withTimeout(INTAKE_MAX_WAIT_TIME),
                new ParallelDeadlineGroup(RC.getDrivetrain().runPathT("TOP-D"), GetCoralStationPieceSecondary(RC)),
                driveAutoAlign(RC, 3, 0.8),
                GoL4Score(RC),

                EnsureNeutralState(RC),
                new WaitCommand(0.32),
                RC.getDrivetrain().runPathT("D-TOP"),
                GetCoralStationPiece(RC).withTimeout(INTAKE_MAX_WAIT_TIME),
                new ParallelDeadlineGroup(RC.getDrivetrain().runPathT("TOP-C"), GetCoralStationPieceSecondary(RC)),
                driveAutoAlign(RC, 2, 0.5),
                GoL4Score(RC),

                EnsureNeutralState(RC),
                new WaitCommand(0.32),
                RC.getDrivetrain().runPathT("C-TOP"),
                GetCoralStationPiece(RC).withTimeout(INTAKE_MAX_WAIT_TIME),
                new ParallelDeadlineGroup(RC.getDrivetrain().runPathT("TOP-B"), GetCoralStationPieceSecondary(RC)),
                driveAutoAlign(RC, 1, 0.5),
                GoL4Score(RC));
    }

    public static Command CoralStationTest(RobotContainer RC) {
        return new SequentialCommandGroup(
                GetCoralStationPiece(RC).withTimeout(INTAKE_MAX_WAIT_TIME),
                new ParallelDeadlineGroup(new WaitCommand(5), GoL2(RC), GetCoralStationPieceSecondary(RC)),
                GoL1(RC));
    }

    public static Command L4ScoreTest(RobotContainer RC) {
        return new SequentialCommandGroup(
                GoL4Score(RC),
                EnsureNeutralState(RC));
    }

    public static Command L4CenterAlgae(RobotContainer RC) {

        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("P-H"),
                driveAutoAlign(RC, 7, 0.5),
                GoL4ScoreSteady(RC),
                RC.getDrivetrain().runPathT("CoralToAlgae"),
                new WaitCommand(0.2),
                SetAlgaeLow(RC),
                new WaitCommand(0.5),
                RC.getDrivetrain().runPathT("RetrieveAlgae"),
                RC.getDrivetrain().runPathT("BackupAlgae"),
                new InstantCommand(() -> RC.getWrist().setWristAngle(CONSTANTS_WRIST.PIVOT_ALGAE_NET))
                        .withTimeout(CONSTANTS_WRIST.WRIST_TIMEOUT),
                RC.getDrivetrain().runPathT("ScoreMidAlgae"),
                RC.getDrivetrain().runPathT("ScoreMidAlgaeSPIN"),
                SetAlgaeNet(RC),
                ScoreAlgae(RC),
                RC.getDrivetrain().runPathT("BargeLittle2"),
                EnsureNeutralState(RC));
    }

    public static Command L4CenterAlgaeTickle(RobotContainer RC) {
        return new SequentialCommandGroup(
                RC.getDrivetrain().runPathT("P-H"),
                driveAutoAlign(RC, 7, 0.5),
                GoL4ScoreSteady(RC),
                RC.getDrivetrain().runPathT("CoralToAlgae"),
                new WaitCommand(0.2),
                SetAlgaeLow(RC),
                new WaitCommand(0.5),
                RC.getDrivetrain().runPathT("RetrieveAlgae"),
                RC.getDrivetrain().runPathT("BackupAlgae"),
                RC.getDrivetrain().runPathT("ScoreMidAlgae"),
                SetAlgaeNet(RC),
                ScoreAlgae(RC),
                EnsureNeutralState(RC),
                RC.getDrivetrain().runPathT("CenterBargeSafe"));
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

    public static Command GetCoralStationPiece(RobotContainer RC) {
        return new SequentialCommandGroup(new PrepIntakeCoralAuto(RC), new PrepCoralLock(RC));
    }

    public static Command GetCoralStationPieceSecondary(RobotContainer RC) {
        return new SequentialCommandGroup(new PrepIntakeCoralAutoSecondary(RC), new PrepCoralLock(RC));
    }

    public static Command GoL4(RobotContainer RC) {
        return new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4))
                .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT);
    }

    public static Command GoL2(RobotContainer RC) {
        return new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L2))
                .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT);
    }

    public static Command GoL1(RobotContainer RC) {
        return new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L1))
                .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT);
    }

    public static Command GoL4Score(RobotContainer RC) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4))
                                .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT),
                        new WaitCommand(0.95)),
                RC.getCoral().outtakeCoralL4Auto().withTimeout(0.15));
    }

    public static Command GoL4ScoreSteady(RobotContainer RC) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4))
                                .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT),
                        new WaitCommand(1.2)),
                RC.getCoral().outtakeCoralL4Auto().withTimeout(0.15));
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

    public static Command SetAlgaeNet(RobotContainer RC) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> RC.getWrist().setWristAngle(CONSTANTS_WRIST.PIVOT_ALGAE_NET))
                        .withTimeout(CONSTANTS_WRIST.WRIST_TIMEOUT),
                new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_NET)),
                new WaitCommand(1.1));
    }

    private static Command ScoreAlgae(RobotContainer RC) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_OUTTAKE_SPEED)),
                new WaitCommand(0.5),
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(0)));
    }

    public static Command EnsureNeutralStateHandler(RobotContainer RC) {
        return new NeutralStateHandler(RC);
    }

    public static Command EnsureNeutralState(RobotContainer RC) {
        return new InstantCommand(() -> new NeutralState(RC).schedule());
    }

    private void configureAutoBindings() {
        // *** NORMIE AUTOS
        autoChooser.addOption("L4FourPieceHigh", L4FourPieceHigh(RC));
        autoChooser.addOption("L4FourPieceLow", L4FourPieceLow(RC));
        autoChooser.addOption("L4CenterAlgae", L4CenterAlgae(RC));

        // *** TICKLE AUTOS
        // autoChooser.addOption("L4FourPieceHighTICKLE", L4FourPieceHighTICKLE(RC));

        // *** TESTER AUTOS
        // autoChooser.addOption("CoralStationTest", CoralStationTest(RC));
        // autoChooser.addOption("L4ScoreTest", L4ScoreTest(RC));
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