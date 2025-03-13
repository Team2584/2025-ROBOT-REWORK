package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CONSTANTS.*;
import frc.robot.commands.NeutralAlgaeState;
import frc.robot.commands.NeutralState;
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
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

public class Autons {
    private static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();

    public static SendableChooser<Command> autoChooser = new SendableChooser<>();

    private static Pose2d[] SELECTED_AUTO_PREP_MAP;
    private static String SELECTED_AUTO_PREP_MAP_NAME = "none";
    private static int AUTO_PREP_NUM = 0;

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

    public static Command CenterAuto(RobotContainer RC) {
        return new SequentialCommandGroup(
                resetSwerveAutoPose(RC, "P-H"),
                followPathCommand("P-H"),
                idleDrive,
                driveAutoAlign(RC, 7),
                TOFDriveScore(RC)

        );

    }

    public static Command followPathCommand(String fileName) {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(fileName);
            

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(fileName));
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    /*
     * This will set the chassis to idle mode
     */
    public static void idleCommandSwerveDrivetrain() {
        commandSwerveDrivetrain.setControl(new SwerveRequest.Idle());
    }

    /*
     * This will destroy our "second chassis" (auton)
     */
    public static void closeCommandSwerveDrivetrain() {
        // TODO: MICHAEL DO WE WANT A NULL HERE?? (think abt this for more than a sec
        // though)
        // commandSwerveDrivetrain = null;
        commandSwerveDrivetrain.close();
    }

    public static Command resetSwerveAutoPose(RobotContainer RC, String nextPath) {
        Rotation2d desiredRotation = Rotation2d.kZero;
        Translation2d desiredPosition = Translation2d.kZero;


        try {
            desiredRotation = PathPlannerPath.fromPathFile(nextPath)
                    .getIdealStartingState().rotation();
            desiredPosition = PathPlannerPath.fromPathFile(nextPath).getWaypoints().get(0).anchor();

            if (CONSTANTS_FIELD.isRedAlliance()) {
                desiredRotation = desiredRotation.plus(Rotation2d.k180deg);
            }
        } catch (Exception e) {
        }

        return AutoBuilder.resetOdom(new Pose2d(desiredPosition, desiredRotation));
    }

    public static Command driveAutoAlign(RobotContainer RC) {
        return Commands.runOnce(() -> RC.getDrivetrain().autoAlign(Meters.of(0),
                SELECTED_AUTO_PREP_MAP[AUTO_PREP_NUM], MetersPerSecond.of(0),
                MetersPerSecond.of(0), DegreesPerSecond.of(0), 1.0, false, Meters.of(1000),
                DriverState.REEF_AUTO_DRIVING,
                DriverState.REEF_AUTO_DRIVING, RC.getState())).repeatedly();
    }

    public static Command driveAutoAlign(RobotContainer RC, int fieldPositionIndex) {
        return Commands.runOnce(() -> RC.getDrivetrain().autoAlign(Meters.of(0),
                CONSTANTS_FIELD.getReefPositions().get().get(fieldPositionIndex), MetersPerSecond.of(0),
                MetersPerSecond.of(0), DegreesPerSecond.of(0), 1.0, false, Meters.of(1000),
                DriverState.REEF_AUTO_DRIVING,
                DriverState.REEF_AUTO_DRIVING, RC.getState())).repeatedly();
    }

    /*
     * This will recreate our "second chassis" (auton)
     */
    public static void openCommandSwerveDrivetrain() {
        commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    }

    // ---** COMMANDS **---

    private static Command idleDrive = new InstantCommand(() -> idleCommandSwerveDrivetrain());
    private static Command openDrive = new InstantCommand(() -> openCommandSwerveDrivetrain());

    private static Command TOFDriveScore(RobotContainer RC) {
        return new TOFDrive(RC, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE)
                .andThen(RC.getCoral().outtakeCoral().withTimeout(0.125));
    }

    private void configureAutoBindings() {

        autoChooser.addOption("IdleAuto", CenterAuto(RC));

        Command driveAutoAlign = Commands.runOnce(() -> RC.getDrivetrain().autoAlign(Meters.of(0),
                SELECTED_AUTO_PREP_MAP[AUTO_PREP_NUM], MetersPerSecond.of(0),
                MetersPerSecond.of(0), DegreesPerSecond.of(0), 1.0, false, Meters.of(1000),
                DriverState.REEF_AUTO_DRIVING,
                DriverState.REEF_AUTO_DRIVING, RC.getState())).repeatedly();

        Command placeSequenceL4 = Commands.sequence(
                driveAutoAlign.asProxy().withTimeout(1), // Attempt to align for up to 1 second
                Commands.runOnce(() -> RC.getDrivetrain().drive(new ChassisSpeeds(), false)), // Stop driving
                new PrepCoralLvl4(RC).asProxy().withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT), // Give it
                                                                                                      // time to
                // "prepare" without
                // checking state
                RC.getCoral().outtakeCoral().asProxy().withTimeout(0.25), // Give it time to "score" without
                                                                          // checking state
                Commands.runOnce(() -> AUTO_PREP_NUM++)); // Increment counter

        NamedCommands.registerCommand("PlaceSequenceL4",
                Commands.sequence(
                        driveAutoAlign.asProxy().withTimeout(1), // Attempt to align for up to 1 second
                        Commands.runOnce(() -> RC.getDrivetrain().drive(new ChassisSpeeds(), false)), // Stop driving
                        new PrepCoralLvl4(RC).asProxy().withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT), // Give it
                                                                                                              // time to
                        // "prepare" without
                        // checking state
                        RC.getCoral().outtakeCoral().asProxy().withTimeout(0.25), // Give it time to "score" without
                                                                                  // checking state
                        Commands.runOnce(() -> AUTO_PREP_NUM++) // Increment counter
                ).withName("PlaceSequence"));

        NamedCommands.registerCommand("autoAlign", Commands.sequence(idleDrive,
                driveAutoAlign.withTimeout(5)));

        NamedCommands.registerCommand("PrepPlace",
                new PrepCoralLvl4(RC).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT)
                        .asProxy().withName("PrepPlace"));

        NamedCommands.registerCommand("GetCoralStationPiece",
                RC.getCoral().intakeCoral().asProxy().until(() -> RC.getCoral().coralLoaded())
                        .withName("GetCoralStationPiece"));

        NamedCommands.registerCommand("prepNet", new PrepNetAlgae(RC).withTimeout(1));

        NamedCommands.registerCommand("wrist60Deg",
                RC.getWrist().setWristAngleCommand(CONSTANTS_WRIST.PIVOT_ALGAE_NEUTRAL)
                        .withTimeout(0.3));

        NamedCommands.registerCommand("shootAlgae", RC.getAlgae().outtakeAlgae());

        NamedCommands.registerCommand("liftLowAlgae", new PickupReefLowAlgae(RC).withTimeout(1));

        NamedCommands.registerCommand("algaeNeutral", new NeutralAlgaeState(RC).withTimeout(1));

        NamedCommands.registerCommand("neutral", new NeutralState(RC).withTimeout(1));

        NamedCommands.registerCommand("liftL4", new PrepCoralLvl4(RC).withTimeout(0.5));

        NamedCommands.registerCommand("liftL3", new PrepCoralLvl3(RC).withTimeout(0.3));

        NamedCommands.registerCommand("scoreCoral", RC.getCoral().outtakeCoral().withTimeout(0.125));

        NamedCommands.registerCommand("liftHighAlgae", new PickupReefHighAlgae(RC).withTimeout(1.5));

        NamedCommands.registerCommand("liftNet", new PrepNetAlgae(RC));

        NamedCommands.registerCommand("scoreNet", RC.getAlgae().outtakeAlgae().withTimeout(0.3));

        NamedCommands.registerCommand("intakeCoral", RC.getCoral().intakeCoral());

        NamedCommands.registerCommand("TOFDrive&Score",
                new TOFDrive(RC, CONSTANTS_DRIVETRAIN.TOF_SPEED, CONSTANTS_DRIVETRAIN.TOF_DISTANCE)
                        .andThen(RC.getCoral().outtakeCoral().withTimeout(0.125)));

        // // -- Event Markers --
        EventTrigger prepPlace = new EventTrigger("PrepPlace");
        prepPlace.onTrue(new PrepCoralLvl4(RC).withTimeout(CONSTANTS_ELEVATOR.ELEVATOR_MAX_TIMEOUT));

        EventTrigger getCoralStationPiece = new EventTrigger("GetCoralStationPiece");
        getCoralStationPiece.onTrue(RC.getCoral().intakeCoral());

        EventTrigger neutral = new EventTrigger("neutral");
        neutral.onTrue(new NeutralState(RC).withTimeout(1));
    }

    public static void runAuton(String auto) { // Autons.runAuton(auto);

    }

    // ---** AUTON INSTANTIATING STUFF **---

    public static Command getAutonomousCommand() {
        selectAutoMap();
        return autoChooser.getSelected();
    }

    private void configureAutoSelector() {
        // TODO: MICHAEL JUST PUT THE EZ AUTON IN THE DEFAULT AUTO HERE
        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData(autoChooser);
    }

    private static void selectAutoMap() {
        SELECTED_AUTO_PREP_MAP = configureAutoPrepMaps(autoChooser.getSelected().getName());
        SELECTED_AUTO_PREP_MAP_NAME = autoChooser.getSelected().getName();
    }

    private static Pose2d[] configureAutoPrepMaps(String selectedAuto) {
        List<Pose2d> fieldPositions = CONSTANTS_FIELD.getReefPositions().get();

        switch (selectedAuto) {
            case "4PIECE_L4_HIGH":
                Pose2d[] PIECE_L4_HIGH = new Pose2d[4];
                PIECE_L4_HIGH[0] = fieldPositions.get(9); // J
                PIECE_L4_HIGH[1] = fieldPositions.get(10); // K
                PIECE_L4_HIGH[2] = fieldPositions.get(11); // L
                PIECE_L4_HIGH[3] = fieldPositions.get(0); // A
                return PIECE_L4_HIGH;
            case "IdleAuto":
                Pose2d[] CenterAuto = new Pose2d[1];
                CenterAuto[0] = fieldPositions.get(7); // H
                return CenterAuto;
            default:
                Pose2d[] noAutoSelected = new Pose2d[1];
                noAutoSelected[0] = new Pose2d();
                return noAutoSelected;
        }
    }
}
