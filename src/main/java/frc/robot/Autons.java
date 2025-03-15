package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.time.Instant;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CONSTANTS.*;
import frc.robot.commands.NeutralAlgaeState;
import frc.robot.commands.NeutralState;
import frc.robot.commands.NeutralStateHandler;
import frc.robot.commands.TOFDrive;
import frc.robot.commands.prep_algae.PickupReefHighAlgae;
import frc.robot.commands.prep_algae.PickupReefLowAlgae;
import frc.robot.commands.prep_algae.PrepNetAlgae;
import frc.robot.commands.prep_coral.PrepCoralLvl3;
import frc.robot.commands.prep_coral.PrepCoralLvl4;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.State.DriverState;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.Drivetrain;

public class Autons {
    private static SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final RobotContainer RC;

    /*
     * Not used because we are calling the class directly
     * (pls only use static)
     */
    public Autons(RobotContainer RC) {
        this.RC = RC;
        configureAutoSelector();
        configureAutoBindings();
    }

    public static Command L4FourPieceHigh(RobotContainer RC) {
        EventTrigger prepPlace = new EventTrigger("prepPlace");
        prepPlace.onTrue(new ParallelCommandGroup(
                new InstantCommand(() -> RC.getElevator().setPosition(CONSTANTS_ELEVATOR.HEIGHT_CORAL_L4))
                        .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT),
                new InstantCommand(() -> RC.getWrist().setWristAngle(CONSTANTS_WRIST.PIVOT_SCORE_CORAL)))
                .withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT));

        EventTrigger getCoralStationPiece = new EventTrigger("getCoralStationPiece");
        getCoralStationPiece.onTrue(RC.getCoral().intakeCoral());

        EventTrigger neutral = new EventTrigger("neutral");
        neutral.onTrue(new NeutralState(RC).withTimeout(1));

        return new SequentialCommandGroup(
                resetToAutoPose(RC, "P-J"),
                RC.getDrivetrain().runPathT("P-J"),
                // PlaceL4Sequence(RC, 9, 0.3),
                driveAutoAlign(RC, 9, 1),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC),
                new WaitCommand(0.2),

                RC.getDrivetrain().runPathT("J-TOP"),
                RC.getDrivetrain().runPathT("TOP-K"),
                // PlaceL4Sequence(RC, 10, 0.3),
                driveAutoAlign(RC, 10, 1),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC),
                new WaitCommand(0.2),

                RC.getDrivetrain().runPathT("K-TOP"),
                RC.getDrivetrain().runPathT("TOP-L"),
                // PlaceL4Sequence(RC, 11, 0.3),
                driveAutoAlign(RC, 11, 1),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC),
                new WaitCommand(0.2),

                RC.getDrivetrain().runPathT("L-TOP"),
                RC.getDrivetrain().runPathT("TOP-A"),
                // PlaceL4Sequence(RC, 0, 0.3)
                driveAutoAlign(RC, 0, 1),
                GoL4(RC),
                new WaitCommand(0.2),
                TOFDriveScore(RC));
    }

    public static Command L4CenterAlgae(RobotContainer RC) {
        EventTrigger pickupLowAlgae = new EventTrigger("pickupLowAlgae");
        pickupLowAlgae.onTrue(new PickupReefLowAlgae(RC).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT));

        EventTrigger pickupHighAlgae = new EventTrigger("pickupHighAlgae");
        pickupLowAlgae.onTrue(new PickupReefHighAlgae(RC).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT));

        EventTrigger NeutralAlgaeState = new EventTrigger("NeutralAlgaeState");
        NeutralAlgaeState.onTrue(new NeutralAlgaeState(RC));

        EventTrigger PepareNetAlgae = new EventTrigger("PepareNetAlgae");
        PepareNetAlgae.onTrue(new PrepNetAlgae(RC));

        return new SequentialCommandGroup(
                resetToAutoPose(RC, "P-H"),
                RC.getDrivetrain().runPathT("P-H"),
                driveAutoAlign(RC, 7, 1),
                GoL4(RC),
                new WaitCommand(0.75),
                TOFDriveScore(RC),
                new WaitCommand(0.5),
                RC.getDrivetrain().runPathT("CoralToAlgae"),
                new WaitCommand(1),
                SetAlgaeLow(RC),
                new WaitCommand(1),
                RC.getDrivetrain().runPathT("RetrieveAlgae"),
                RC.getDrivetrain().runPathT("BackupAlgae"),
                RC.getDrivetrain().runPathT("ScoreMidAlgae"),
                new WaitCommand(1.5),
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(CONSTANTS_ALGAE.ALGAE_OUTTAKE_SPEED)),
                new WaitCommand(0.5),
                new InstantCommand(() -> RC.getAlgae().setAlgaeIntakeMotor(0)),
                RC.getDrivetrain().runPathT("Barge-Algae-I")
        // SetAlgaeHigh(RC),
        // RC.getDrivetrain().runPathT("Algae-I-Backup")
        );
    }

    public static Command L4OnePieceLow(RobotContainer RC) {

        return new SequentialCommandGroup(
            resetToAutoPose(RC, "P-E"),
            RC.getDrivetrain().runPathT("P-E"),
            driveAutoAlign(RC, 7, 1),
            GoL4(RC),
            new WaitCommand(0.75),
            TOFDriveScore(RC),
            new WaitCommand(0.5)
        );
    }

    public static Command resetToAutoPose(RobotContainer RC, String nextPath) {
        Rotation2d desiredRotation = Rotation2d.kZero;
        Translation2d desiredPosition = Translation2d.kZero;

        try {
            desiredRotation = PathPlannerPath.fromPathFile(nextPath)
                    .getIdealStartingState().rotation();
            desiredPosition = PathPlannerPath.fromPathFile(nextPath).getWaypoints().get(0).anchor();
            // if (CONSTANTS_FIELD.isRedAlliance()) {
            // // desiredRotation = desiredRotation.plus(Rotation2d.k180deg);
            // }
        } catch (Exception e) {
        }

        // RC.getDrivetrain().resetYaw(desiredRotation.getDegrees());
        // RC.getDrivetrain().resetPoseToPose(new
        // Pose2d(RC.getDrivetrain().getPose().getTranslation(), desiredRotation));
        // RC.getDrivetrain().resetPoseToPose(new Pose2d(desiredPosition,
        // desiredRotation));

        return new InstantCommand();
    }

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
                new TOFDrive(RC, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE)
                        .andThen(RC.getCoral().outtakeCoral().withTimeout(0.125)));
    }

    public static Command GetCoralStationPiece(RobotContainer RC) {
        return RC.getCoral().intakeCoral().asProxy();
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
                RC.getAlgae().intakeAlgae());
    }

    // ---** COMMANDS **---

    private static Command TOFDriveScore(RobotContainer RC) {
        return new TOFDrive(RC, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE)
                .andThen(RC.getCoral().outtakeCoral().withTimeout(0.125));
    }

    private void configureAutoBindings() {

        autoChooser.addOption("L4FourPieceHigh", L4FourPieceHigh(RC));
        autoChooser.addOption("L4CenterAlgae", L4CenterAlgae(RC));

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