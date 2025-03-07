package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import com.frcteam3255.components.swerve.SN_SwerveModule;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_DRIVETRAIN;
import frc.robot.CONSTANTS.CONSTANTS_FIELD;
import frc.robot.CONSTANTS.CONSTANTS_VISION;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;
import frc.robot.subsystems.State;
import frc.robot.subsystems.State.DriverState;

@Logged
public class Drivetrain extends Swerve {
        Pose2d desiredAlignmentPose = Pose2d.kZero;
        SwerveModuleState[] desiredModuleStates;
        SwerveModuleState[] actualModuleStates;

        public Drivetrain() {
                super(
                                CONSTANTS_DRIVETRAIN.SWERVE_CONSTANTS,
                                CONSTANTS_DRIVETRAIN.MODULES,
                                CONSTANTS_DRIVETRAIN.WHEEL_DISTANCE_LENGTH,
                                CONSTANTS_DRIVETRAIN.WHEEL_DISTANCE_WIDTH,
                                CONSTANTS_PORTS.CAN_BUS_NAME,
                                CONSTANTS_PORTS.PIGEON_CAN,
                                CONSTANTS_DRIVETRAIN.MIN_STEER_PERCENT,
                                CONSTANTS_DRIVETRAIN.INVERSION_LEFT,
                                CONSTANTS_DRIVETRAIN.INVERSION_RIGHT,
                                CONSTANTS_DRIVETRAIN.STEER_MOTOR_INVERT,
                                CONSTANTS_DRIVETRAIN.CANCODER_INVERT,
                                CONSTANTS_DRIVETRAIN.DRIVE_NEUTRAL_MODE,
                                CONSTANTS_DRIVETRAIN.STEER_NEUTRAL_MODE,
                                VecBuilder.fill(
                                                CONSTANTS_DRIVETRAIN.MEASUREMENT_STD_DEVS_POS,
                                                CONSTANTS_DRIVETRAIN.MEASUREMENT_STD_DEVS_POS,
                                                CONSTANTS_DRIVETRAIN.MEASUREMENT_STD_DEV_HEADING),
                                VecBuilder.fill(
                                                CONSTANTS_VISION.MEGA_TAG2_STD_DEVS_POSITION,
                                                CONSTANTS_VISION.MEGA_TAG2_STD_DEVS_POSITION,
                                                CONSTANTS_VISION.MEGA_TAG2_STD_DEVS_HEADING),
                                CONSTANTS_DRIVETRAIN.AUTO.AUTO_DRIVE_PID,
                                CONSTANTS_DRIVETRAIN.AUTO.AUTO_STEER_PID,
                                CONSTANTS_DRIVETRAIN.AUTO.ROBOT_CONFIG,
                                () -> CONSTANTS_FIELD.isRedAlliance(),
                                Robot.isSimulation());
        }

        @Override
        public void configure() {
                CONSTANTS_DRIVETRAIN.MODULES[0].driveConfiguration = CONSTANTS_DRIVETRAIN.DRIVE_LEFT_CONFIG;
                CONSTANTS_DRIVETRAIN.MODULES[2].driveConfiguration = CONSTANTS_DRIVETRAIN.DRIVE_LEFT_CONFIG;
                CONSTANTS_DRIVETRAIN.MODULES[1].driveConfiguration = CONSTANTS_DRIVETRAIN.DRIVE_RIGHT_CONFIG;
                CONSTANTS_DRIVETRAIN.MODULES[3].driveConfiguration = CONSTANTS_DRIVETRAIN.DRIVE_RIGHT_CONFIG;

                Module.steerConfiguration = CONSTANTS_DRIVETRAIN.STEER_CONFIG;
                Module.cancoderConfiguration = CONSTANTS_DRIVETRAIN.CANCODER_CONFIG;
                Module.cancoderConfiguration = CONSTANTS_DRIVETRAIN.CANCODER_CONFIG;
                super.configure();
        }

        public void addEventToAutoMap(String key, Command command) {
                super.autoEventMap.put(key, command);
        }

        /**
         * Returns the rotational velocity calculated with PID control to reach the
         * given rotation. This must be called every loop until you reach the given
         * rotation.
         * 
         * @param desiredYaw The desired yaw to rotate to
         * @return The desired velocity needed to rotate to that position.
         */
        public AngularVelocity getVelocityToRotate(Rotation2d desiredYaw) {
                double yawSetpoint = CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER
                                .getThetaController()
                                .calculate(getRotation().getRadians(), desiredYaw.getRadians());

                // limit the PID output to our maximum rotational speed
                yawSetpoint = MathUtil.clamp(yawSetpoint, -CONSTANTS_DRIVETRAIN.TURN_SPEED.in(Units.RadiansPerSecond),
                                CONSTANTS_DRIVETRAIN.TURN_SPEED.in(Units.RadiansPerSecond));

                return Units.RadiansPerSecond.of(yawSetpoint);
        }

        /**
         * Returns the rotational velocity calculated with PID control to reach the
         * given rotation. This must be called every loop until you reach the given
         * rotation.
         * 
         * @param desiredYaw The desired yaw to rotate to
         * @return The desired velocity needed to rotate to that position.
         */
        public AngularVelocity getVelocityToRotate(Angle desiredYaw) {
                return getVelocityToRotate(Rotation2d.fromDegrees(desiredYaw.in(Units.Degrees)));
        }

        public Angle getRotationMeasure() {
                return Units.Degrees.of(getRotation().getDegrees());
        }

        /**
         * Calculate the ChassisSpeeds needed to align the robot to the desired pose.
         * This must be called every loop until you reach the desired pose.
         * 
         * @param desiredPose The desired pose to align to
         * @return The ChassisSpeeds needed to align the robot to the desired pose
         */
        public ChassisSpeeds getAlignmentSpeeds(Pose2d desiredPose) {
                desiredAlignmentPose = desiredPose;
                // TODO: This might run better if instead of 0, we use
                // *********************************
                // CONSTANTS_FIELD.TELEOP_AUTO_ALIGN.DESIRED_AUTO_ALIGN_SPEED.in(Units.MetersPerSecond);.
                // I dont know why. it might though
                return CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.calculate(getPose(),
                                desiredPose, 0,
                                desiredPose.getRotation());
        }

        /**
         * Returns the closest reef branch to the robot.
         * 
         * @param leftBranchRequested If we are requesting to align to the left or right
         *                            branch
         * @return The desired reef branch face to align to
         */
        public Pose2d getDesiredReef(boolean leftBranchRequested) {
                // Get the closest reef branch face using either branch on the face
                List<Pose2d> reefPoses = CONSTANTS_FIELD.getReefPositions().get();
                Pose2d currentPose = getPose();
                Pose2d desiredReef = currentPose.nearest(reefPoses);
                int closestReefIndex = reefPoses.indexOf(desiredReef);

                // Invert faces on the back of the reef so they're always relative to the driver
                if (closestReefIndex > 3 && closestReefIndex < 10) {
                        leftBranchRequested = !leftBranchRequested;
                }

                // If we were closer to the left branch but selected the right branch (or
                // vice-versa), switch to our desired branch
                if (leftBranchRequested && (closestReefIndex % 2 == 1)) {
                        desiredReef = reefPoses.get(closestReefIndex - 1);
                } else if (!leftBranchRequested && (closestReefIndex % 2 == 0)) {
                        desiredReef = reefPoses.get(closestReefIndex + 1);
                }
                return desiredReef;
        }

        //
        public Pose2d getDesiredCoralStation(boolean farCoralStationRequested) {
                // Get the closest coral station
                List<Pose2d> coralStationPoses = CONSTANTS_FIELD.getCoralStationPositions().get();
                Pose2d currentPose = getPose();
                Pose2d desiredCoralStation = currentPose.nearest(coralStationPoses);
                int closestCoralStationIndex = coralStationPoses.indexOf(desiredCoralStation);

                // If we were closer to the left branch but selected the right branch (or
                // vice-versa), switch to our desired branch
                if (farCoralStationRequested && (closestCoralStationIndex % 2 == 1)) {
                        desiredCoralStation = coralStationPoses.get(closestCoralStationIndex - 1);
                } else if (!farCoralStationRequested && (closestCoralStationIndex % 2 == 0)) {
                        desiredCoralStation = coralStationPoses.get(closestCoralStationIndex + 1);
                }

                return desiredCoralStation;
        }

        public Pose2d getDesiredProcessor() {
                // Get the closest processor
                List<Pose2d> processorPoses = CONSTANTS_FIELD.getProcessorPositions().get();
                Pose2d currentPose = getPose();
                Pose2d desiredProcessor = currentPose.nearest(processorPoses);

                return desiredProcessor;
        }

        /**
         * Drive the drivetrain with pre-calculated ChassisSpeeds
         *
         * @param chassisSpeeds Desired ChassisSpeeds
         * @param isOpenLoop    Are the modules being set based on open loop or closed
         *                      loop control
         *
         */
        public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
                SwerveModuleState[] desiredModuleStates = swerveKinematics
                                .toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
                // SwerveModuleState[] desiredModuleStates =
                // swerveKinematics.toSwerveModuleStates(chassisSpeeds);
                setModuleStates(desiredModuleStates, isOpenLoop);
        }

        public void rotationalAutoAlign(Distance distanceFromTarget, Pose2d desiredTarget,
                        LinearVelocity xVelocity,
                        LinearVelocity yVelocity,
                        AngularVelocity rVelocity, double elevatorMultiplier, boolean isOpenLoop,
                        Distance maxAutoDriveDistance,
                        DriverState driving, DriverState rotating, State subStateMachine) {

                int redAllianceMultiplier = CONSTANTS_FIELD.isRedAlliance() ? -1 : 1;

                // Rotational-only auto-align
                drive(
                                new Translation2d(xVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond),
                                                yVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond)),
                                getVelocityToRotate(desiredTarget.getRotation()).in(Units.RadiansPerSecond),
                                isOpenLoop);
                subStateMachine.setDriverState(rotating);
        }

        /**
         * Contains logic for automatically aligning & automatically driving to the
         * reef.
         * May align only rotationally, automatically drive to a branch, or be
         * overridden by the driver
         */

        public void autoAlign(Distance distanceFromTarget, Pose2d desiredTarget,
                        LinearVelocity xVelocity,
                        LinearVelocity yVelocity,
                        AngularVelocity rVelocity, double elevatorMultiplier, boolean isOpenLoop,
                        Distance maxAutoDriveDistance,
                        DriverState driving, DriverState rotating, State subStateMachine) {
                desiredAlignmentPose = desiredTarget;
                int redAllianceMultiplier = CONSTANTS_FIELD.isRedAlliance() ? -1 : 1;

                if (distanceFromTarget.gte(maxAutoDriveDistance)) {
                        // Rotational-only auto-align
                        drive(
                                        new Translation2d(
                                                        xVelocity.times(redAllianceMultiplier)
                                                                        .in(Units.MetersPerSecond),
                                                        yVelocity.times(redAllianceMultiplier)
                                                                        .in(Units.MetersPerSecond)),
                                        getVelocityToRotate(desiredTarget.getRotation()).in(Units.RadiansPerSecond),
                                        isOpenLoop);
                        subStateMachine.setDriverState(rotating);
                } else {
                        // Full auto-align
                        ChassisSpeeds desiredChassisSpeeds = getAlignmentSpeeds(desiredTarget);
                        subStateMachine.setDriverState(driving);

                        // Speed limit based on elevator height
                        LinearVelocity linearSpeedLimit = CONSTANTS_DRIVETRAIN.MAX_DRIVE_SPEED
                                        .times(elevatorMultiplier);
                        AngularVelocity angularSpeedLimit = CONSTANTS_DRIVETRAIN.TURN_SPEED.times(elevatorMultiplier);

                        if (!RobotState.isAutonomous()) {
                                if ((desiredChassisSpeeds.vxMetersPerSecond > linearSpeedLimit
                                                .in(Units.MetersPerSecond))
                                                || (desiredChassisSpeeds.vyMetersPerSecond > linearSpeedLimit
                                                                .in(Units.MetersPerSecond))
                                                || (desiredChassisSpeeds.omegaRadiansPerSecond > angularSpeedLimit
                                                                .in(Units.RadiansPerSecond))) {

                                        desiredChassisSpeeds.vxMetersPerSecond = MathUtil.clamp(
                                                        desiredChassisSpeeds.vxMetersPerSecond, 0,
                                                        linearSpeedLimit.in(MetersPerSecond));
                                        desiredChassisSpeeds.vyMetersPerSecond = MathUtil.clamp(
                                                        desiredChassisSpeeds.vyMetersPerSecond, 0,
                                                        linearSpeedLimit.in(MetersPerSecond));
                                        desiredChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
                                                        desiredChassisSpeeds.omegaRadiansPerSecond, 0,
                                                        angularSpeedLimit.in(RadiansPerSecond));
                                }
                        }

                        drive(desiredChassisSpeeds, isOpenLoop);
                }
        }

        public boolean isAtRotation(Rotation2d desiredRotation) {
                return (getRotation().getMeasure()
                                .compareTo(desiredRotation.getMeasure()
                                                .minus(CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) > 0)
                                &&
                                getRotation().getMeasure()
                                                .compareTo(desiredRotation.getMeasure()
                                                                .plus(CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) < 0;
        }

        public boolean isAtPosition(Pose2d desiredPose2d) {
                return Units.Meters
                                .of(getPose().getTranslation().getDistance(desiredPose2d.getTranslation()))
                                .lte(CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.AT_POINT_TOLERANCE);
        }

        @NotLogged
        public Boolean isAligned() {
                return (desiredAlignmentPose.getTranslation().getDistance(
                                getPose().getTranslation()) <= CONSTANTS_DRIVETRAIN.TELEOP_AUTO_ALIGN.AUTO_ALIGNMENT_TOLERANCE
                                                .in(Units.Meters))
                                && isAtRotation(desiredAlignmentPose.getRotation());
        }

        public boolean atPose(Pose2d desiredPose) {
                return isAtRotation(desiredPose.getRotation()) && isAtPosition(desiredPose);
        }

        @Override
        public void periodic() {
                super.periodic();

                field.setRobotPose(getPose());
                desiredModuleStates = getDesiredModuleStates();
                actualModuleStates = getActualModuleStates();

                SmartDashboard.putData(field);
                SmartDashboard.putNumber("Drivetrain/Rotation", getRotationMeasure().in(Units.Degrees));

                for (Module mod : CONSTANTS_DRIVETRAIN.MODULES) {
                        SmartDashboard.putNumber("Drivetrain/Module #" + mod.getModuleNumber() + "/rawAbsoluteEncoder",
                                        (mod.getRawAbsoluteEncoder()));
                        SmartDashboard.putNumber(
                                        "Drivetrain/Module #" + mod.getModuleNumber() + "/absoluteEncoderOffset",
                                        (mod.getAbsoluteEncoderOffset()));
                        SmartDashboard.putNumber("Drivetrain/Module #" + mod.getModuleNumber() + "/plsbeZero",
                                        (mod.getRawAbsoluteEncoder() - mod.getAbsoluteEncoderOffset()));
                        SmartDashboard.putNumber("Drivetrain/Module #" + mod.getModuleNumber() + "/absoluteEncoderConv",
                                        (mod.getAbsoluteEncoder()));
                        SmartDashboard.putNumber(
                                        "Drivetrain/Module #" + mod.getModuleNumber() + "/Angle Desired (Degrees)",
                                        getDesiredModuleStates()[mod.getModuleNumber()].angle.getDegrees());
                        SmartDashboard.putNumber(
                                        "Drivetrain/Module #" + mod.getModuleNumber() + "/Angle Actual (Degrees)",
                                        getActualModuleStates()[mod.getModuleNumber()].angle.getDegrees());
                        SmartDashboard.putNumber("Drivetrain/Module #" + mod.getModuleNumber() + "/Actual Speed (m.s)",
                                        Math.abs(getActualModuleStates()[mod.getModuleNumber()].speedMetersPerSecond));
                        SmartDashboard.putNumber("Drivetrain/Module #" + mod.getModuleNumber() + "/SteerPos",
                                        mod.getAdjustedSteerPositionDouble());

                }
                resetModulesToAbsolute();
        }
}
