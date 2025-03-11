package frc.robot;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.Module;

public final class CONSTANTS {

    public static class CONSTANTS_PORTS {
        public static final String CAN_BUS_NAME = "";

        // Controller
        public static final int CONTROLLER_PORT = 0;
        public static final int BUTTON_BOARD_PORT = 2;

        // Module 0 -- Front Left
        public static final int FRONT_LEFT_DRIVE_CAN = 1;
        public static final int FRONT_LEFT_STEER_CAN = 2;
        public static final int FRONT_LEFT_ABSOLUTE_ENCODER_CAN = 3;

        // Module 1 -- Front Right
        public static final int FRONT_RIGHT_DRIVE_CAN = 7;
        public static final int FRONT_RIGHT_STEER_CAN = 8;
        public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_CAN = 9;

        // Module 2 -- Back Left
        public static final int BACK_LEFT_DRIVE_CAN = 4;
        public static final int BACK_LEFT_STEER_CAN = 5;
        public static final int BACK_LEFT_ABSOLUTE_ENCODER_CAN = 6;

        // Module 3 -- Back Right
        public static final int BACK_RIGHT_DRIVE_CAN = 10;
        public static final int BACK_RIGHT_STEER_CAN = 11;
        public static final int BACK_RIGHT_ABSOLUTE_ENCODER_CAN = 12;

        // Other
        public static final int PIGEON_CAN = 6;
        public static final int TOF_CAN = 30;

        // Elevator
        public static final int ELEVATOR_LEFT_CAN = 21;
        public static final int ELEVATOR_RIGHT_CAN = 20;
        public static final int ELEVATOR_LIMIT_CHANNEL = 9;

        // Wrist
        public static final int WRIST_CAN = 16;
        public static final int WRIST_ENCODER_CAN = 25;

        // Algae
        public static final int ALGAE_CAN = 17;

        // Coral
        public static final int CORAL_CAN = 23;
        public static final int CORAL_SENSOR_CAN = 26;
        public static final int CORAL_ELEVATOR_SENSOR_CHANNEL = 0;

        // Climb
        public static final int CLIMB_CAN = 15;

        // Ramp
        public static final int RAMP_CAN = 28;
    }

    public static class CONSTANTS_CONTROLLER {
        public static final double CONTROLLER_DEADZONE = 0.1;
        public static final boolean SILENCE_JOYSTICK_WARNINGS = true;
    }

    public static class CONSTANTS_DRIVETRAIN {
        public static final double WHEEL_DIAMETER = Units.Inches.of(2).in(Units.Meters);
        public static final Distance WHEEL_RADIUS = Units.Meters.of(WHEEL_DIAMETER / 2);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final LinearVelocity MAX_DRIVE_SPEED = Units.MetersPerSecond.of(5.5);

        public static final double MaxAngularRate = RotationsPerSecond.of(1 * Math.PI).in(RadiansPerSecond);

        // Inverted states
        public static final boolean RIGHT_INVERTED = false;
        public static final boolean LEFT_INVERTED = true;

        public static final boolean FRONT_LEFT_STEER_INVERTED = true;
        public static final boolean FRONT_RIGHT_STEER_INVERTED = true;
        public static final boolean BACK_LEFT_STEER_INVERTED = true;
        public static final boolean BACK_RIGHT_STEER_INVERTED = true;

        public static final boolean FRONT_RIGHT_ABS_ENCODER_INVERTED = false;
        public static final boolean FRONT_LEFT_ABS_ENCODER_INVERTED = false;
        public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_INVERTED = false;
        public static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_INVERTED = false;

        // Encoder offsets (check raw values when all wheel are aligned in the same
        // direction)
        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = -0.1389;
        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = 0.4995;
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET = -0.2412;
        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = -0.0327;

        public static final InvertedValue INVERSION_LEFT = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue INVERSION_RIGHT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue STEER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        private static final double kDriveGearRatio = 5.6;
        private static final double kSteerGearRatio = 13.371428571428572;

        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants(
                CONSTANTS_DRIVETRAIN.kSteerGearRatio, CONSTANTS_DRIVETRAIN.WHEEL_CIRCUMFERENCE,
                CONSTANTS_DRIVETRAIN.kDriveGearRatio, edu.wpi.first.math.util.Units.feetToMeters(17.5));

        public static final double kCancoderBootAllowanceSeconds = 5;

        /*
         * Physically measured from center to center of the wheels
         * Distance between Left & Right Wheels (IN METERS)
         */
        public static final double WHEEL_DISTANCE_WIDTH = 0.58;
        // Distance between Front & Back Wheels
        public static final double WHEEL_DISTANCE_LENGTH = 0.58;

        // PID is set to each module INDIVIDUALLY
        public static final double DRIVE_P = 0.2;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0;

        public static final double STEER_P = 60;
        public static final double STEER_I = 0.0;
        public static final double STEER_D = 0.1;

        public static final double STEER_KS = 0.0;
        public static final double STEER_KV = 0.0;

        public static final double DRIVE_KS = 0;
        public static final double DRIVE_KA = 0;
        public static final double DRIVE_KV = (1 / MAX_DRIVE_SPEED.in(Units.MetersPerSecond));

        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final Current DRIVE_CURRENT_LIMIT = Units.Amps.of(60);

        public static final TalonFXConfiguration DRIVE_LEFT_CONFIG = new TalonFXConfiguration();
        public static final TalonFXConfiguration DRIVE_RIGHT_CONFIG = new TalonFXConfiguration();

        public static final TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();
        public static final CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

        static {
            // Passed into drive constructor
            DRIVE_LEFT_CONFIG.Slot0.kP = DRIVE_P;
            DRIVE_LEFT_CONFIG.Slot0.kI = DRIVE_I;
            DRIVE_LEFT_CONFIG.Slot0.kD = DRIVE_D;
            DRIVE_LEFT_CONFIG.MotorOutput.Inverted = INVERSION_LEFT;
            DRIVE_LEFT_CONFIG.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;
            DRIVE_LEFT_CONFIG.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.driveGearRatio;
            DRIVE_LEFT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = false;
            DRIVE_LEFT_CONFIG.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT.in(Units.Amps);

            DRIVE_RIGHT_CONFIG.Slot0.kP = DRIVE_P;
            DRIVE_RIGHT_CONFIG.Slot0.kI = DRIVE_I;
            DRIVE_RIGHT_CONFIG.Slot0.kD = DRIVE_D;

            DRIVE_RIGHT_CONFIG.MotorOutput.Inverted = INVERSION_RIGHT;
            DRIVE_RIGHT_CONFIG.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;
            DRIVE_RIGHT_CONFIG.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.driveGearRatio;
            DRIVE_RIGHT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = false;
            DRIVE_RIGHT_CONFIG.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT.in(Units.Amps);

            STEER_CONFIG.Slot0.kP = STEER_P;
            STEER_CONFIG.Slot0.kI = STEER_I;
            STEER_CONFIG.Slot0.kD = STEER_D;
            STEER_CONFIG.Slot0.kV = STEER_KV;
            STEER_CONFIG.Slot0.kS = STEER_KS;
            STEER_CONFIG.MotorOutput.Inverted = STEER_MOTOR_INVERT;
            STEER_CONFIG.MotorOutput.NeutralMode = STEER_NEUTRAL_MODE;
            STEER_CONFIG.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.steerGearRatio;
            STEER_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;

            CANCODER_CONFIG.MagnetSensor.SensorDirection = CANCODER_INVERT;
        }

        public static final double MIN_STEER_PERCENT = 0.01;
        public static final double SLOW_MODE_GOVERNOR = 0.5; // slow mode speed limiter
        public static final double MINIMUM_ELEVATOR_GOVERNOR = 0.1; // elevator up drive speed limiter

        // One spin per second (for teleop)
        public static final AngularVelocity TURN_SPEED = Units.DegreesPerSecond.of(360);

        /**
         * <p>
         * Pose estimator standard deviation for encoder & gyro data
         * </p>
         * <b>Units:</b> Meters
         */
        public static final double MEASUREMENT_STD_DEVS_POS = 0.05;

        /**
         * <p>
         * Pose estimator standard deviation for encoder & gyro data
         * </p>
         * <b>Units:</b> Radians
         */
        public static final double MEASUREMENT_STD_DEV_HEADING = Units.Radians.convertFrom(4, Units.Degrees);

        public static Module[] MODULES = new Module[] {
                // Front Left
                new Module(0, SWERVE_CONSTANTS, CONSTANTS_PORTS.FRONT_LEFT_DRIVE_CAN,
                        CONSTANTS_PORTS.FRONT_LEFT_STEER_CAN,
                        CONSTANTS_PORTS.FRONT_LEFT_ABSOLUTE_ENCODER_CAN,
                        CONSTANTS_DRIVETRAIN.FRONT_LEFT_ABS_ENCODER_OFFSET, CONSTANTS_DRIVETRAIN.INVERSION_LEFT,
                        CONSTANTS_DRIVETRAIN.DRIVE_LEFT_CONFIG,
                        CONSTANTS_PORTS.CAN_BUS_NAME),

                // Front Right
                new Module(1, SWERVE_CONSTANTS, CONSTANTS_PORTS.FRONT_RIGHT_DRIVE_CAN,
                        CONSTANTS_PORTS.FRONT_RIGHT_STEER_CAN,
                        CONSTANTS_PORTS.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN,
                        CONSTANTS_DRIVETRAIN.FRONT_RIGHT_ABS_ENCODER_OFFSET, CONSTANTS_DRIVETRAIN.INVERSION_RIGHT,
                        CONSTANTS_DRIVETRAIN.DRIVE_RIGHT_CONFIG,
                        CONSTANTS_PORTS.CAN_BUS_NAME),

                // Back Left
                new Module(2, SWERVE_CONSTANTS, CONSTANTS_PORTS.BACK_LEFT_DRIVE_CAN,
                        CONSTANTS_PORTS.BACK_LEFT_STEER_CAN,
                        CONSTANTS_PORTS.BACK_LEFT_ABSOLUTE_ENCODER_CAN,
                        CONSTANTS_DRIVETRAIN.BACK_LEFT_ABS_ENCODER_OFFSET, CONSTANTS_DRIVETRAIN.INVERSION_LEFT,
                        CONSTANTS_DRIVETRAIN.DRIVE_LEFT_CONFIG,
                        CONSTANTS_PORTS.CAN_BUS_NAME),

                // Back Right
                new Module(3, SWERVE_CONSTANTS, CONSTANTS_PORTS.BACK_RIGHT_DRIVE_CAN,
                        CONSTANTS_PORTS.BACK_RIGHT_STEER_CAN,
                        CONSTANTS_PORTS.BACK_RIGHT_ABSOLUTE_ENCODER_CAN,
                        CONSTANTS_DRIVETRAIN.BACK_RIGHT_ABS_ENCODER_OFFSET, CONSTANTS_DRIVETRAIN.INVERSION_RIGHT,
                        CONSTANTS_DRIVETRAIN.DRIVE_RIGHT_CONFIG,
                        CONSTANTS_PORTS.CAN_BUS_NAME),
        };

        public static class AUTO {
            // This PID is implemented on the Drivetrain subsystem
            public static final double AUTO_DRIVE_P = 9;
            public static final double AUTO_DRIVE_I = 0;
            public static final double AUTO_DRIVE_D = 0;
            public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(
                    CONSTANTS_DRIVETRAIN.AUTO.AUTO_DRIVE_P,
                    CONSTANTS_DRIVETRAIN.AUTO.AUTO_DRIVE_I,
                    CONSTANTS_DRIVETRAIN.AUTO.AUTO_DRIVE_D);

            public static final double AUTO_STEER_P = 5.7;
            public static final double AUTO_STEER_I = 0.0;
            public static final double AUTO_STEER_D = 0.0;
            public static final PIDConstants AUTO_STEER_PID = new PIDConstants(
                    CONSTANTS_DRIVETRAIN.AUTO.AUTO_STEER_P,
                    CONSTANTS_DRIVETRAIN.AUTO.AUTO_STEER_I,
                    CONSTANTS_DRIVETRAIN.AUTO.AUTO_STEER_D);

            public static final Mass MASS = Units.Kilograms.of(51);
            // TODO: Calculate real MOI
            public static final double MOI = 5.0;
            public static final double WHEEL_COF = 1.2;
            public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio);
            public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(WHEEL_RADIUS, MAX_DRIVE_SPEED,
                    WHEEL_COF,
                    DRIVE_MOTOR,
                    DRIVE_CURRENT_LIMIT, 1);

            public static final Translation2d[] MODULE_OFFSETS = {
                    new Translation2d(WHEEL_DISTANCE_LENGTH / 2.0, WHEEL_DISTANCE_WIDTH / 2.0),
                    new Translation2d(WHEEL_DISTANCE_LENGTH / 2.0, -WHEEL_DISTANCE_WIDTH / 2.0),
                    new Translation2d(-WHEEL_DISTANCE_LENGTH / 2.0, WHEEL_DISTANCE_WIDTH / 2.0),
                    new Translation2d(-WHEEL_DISTANCE_LENGTH / 2.0, -WHEEL_DISTANCE_WIDTH / 2.0) };

            public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS.in(Kilograms), MOI, MODULE_CONFIG,
                    MODULE_OFFSETS);

        }

        public static class TELEOP_AUTO_ALIGN {
            public static final LinearVelocity DESIRED_AUTO_ALIGN_SPEED = Units.MetersPerSecond
                    .of(CONSTANTS_DRIVETRAIN.MAX_DRIVE_SPEED.in(MetersPerSecond) / 4.5);

            public static final Distance MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE = Units.Meters.of(15);
            public static final Distance MAX_AUTO_DRIVE_REEF_DISTANCE = Units.Meters.of(3);
            public static final Distance MAX_AUTO_DRIVE_PROCESSOR_DISTANCE = Units.Meters.of(8);
            public static final LinearVelocity MIN_DRIVER_OVERRIDE = CONSTANTS_DRIVETRAIN.MAX_DRIVE_SPEED.div(10);

            public static final PIDController PID_TRANSLATION = new PIDController(
                    1.4,
                    0,
                    0.0);
            public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.5);

            public static final ProfiledPIDController PID_ROTATIONAL = new ProfiledPIDController(
                    3, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
                            Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));
            public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(3);

            public static final Distance AUTO_ALIGNMENT_TOLERANCE = Units.Inches.of(0.7);

            static {
                PID_TRANSLATION.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

                PID_ROTATIONAL.enableContinuousInput(0, 360);
                PID_ROTATIONAL.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
            }

            public static HolonomicDriveController TELEOP_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
                    PID_TRANSLATION,
                    PID_TRANSLATION,
                    PID_ROTATIONAL);
        }
    }

    public static class CONSTANTS_ELEVATOR {
        public static final Distance ELEVATOR_PULLEY_PITCH_DIAMETER = Units.Inches.of(1.504);
        // /3.0 for modified elevator ratio (~1.0s -> ~0.3s elevator max extention time)
        public static final double ELEVATOR_GEAR_RATIO = 8.571 / 3.0;

        // Elevator UP
        public static TalonFXConfiguration ELEVATOR_CONFIG_0 = new TalonFXConfiguration();
        // Elevator DOWN
        public static TalonFXConfiguration ELEVATOR_CONFIG_1 = new TalonFXConfiguration();

        static {
            ELEVATOR_CONFIG_0.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG_0.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            ELEVATOR_CONFIG_1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG_1.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            ELEVATOR_CONFIG_0.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ELEVATOR_CONFIG_0.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(54).in(Units.Inches);
            ELEVATOR_CONFIG_0.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ELEVATOR_CONFIG_0.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0)
                    .in(Units.Inches);

            ELEVATOR_CONFIG_1.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ELEVATOR_CONFIG_1.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(54).in(Units.Inches);
            ELEVATOR_CONFIG_1.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ELEVATOR_CONFIG_1.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0)
                    .in(Units.Inches);

            ELEVATOR_CONFIG_0.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            ELEVATOR_CONFIG_1.Slot0.GravityType = GravityTypeValue.Elevator_Static;

            // Elevator motors will provide feedback in INCHES the carriage has moved
            ELEVATOR_CONFIG_0.Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_RATIO;
            ELEVATOR_CONFIG_1.Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_RATIO;

            ELEVATOR_CONFIG_0.Slot0.kG = 0.0; // Volts to overcome gravity
            ELEVATOR_CONFIG_0.Slot0.kS = 0.3; // Volts to overcome static friction
            ELEVATOR_CONFIG_0.Slot0.kV = 0.3; // Volts for a velocity target of 1 rps
            ELEVATOR_CONFIG_0.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
            ELEVATOR_CONFIG_0.Slot0.kP = 13;
            ELEVATOR_CONFIG_0.Slot0.kI = 0.0;
            ELEVATOR_CONFIG_0.Slot0.kD = 0.01;

            ELEVATOR_CONFIG_1.Slot0.kG = 0.0; // Volts to overcome gravity
            ELEVATOR_CONFIG_1.Slot0.kS = 0.3; // Volts to overcome static friction
            ELEVATOR_CONFIG_1.Slot0.kV = 0.3; // Volts for a velocity target of 1 rps
            ELEVATOR_CONFIG_1.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
            ELEVATOR_CONFIG_1.Slot0.kP = 2;
            ELEVATOR_CONFIG_1.Slot0.kI = 0.0;
            ELEVATOR_CONFIG_1.Slot0.kD = 0.01;

            // ELEVATOR_CONFIG.Slot0.StaticFeedforwardSign =
            // StaticFeedforwardSignValue.UseClosedLoopSign;

            ELEVATOR_CONFIG_0.MotionMagic.MotionMagicCruiseVelocity = 100;
            ELEVATOR_CONFIG_0.MotionMagic.MotionMagicAcceleration = 65;
            ELEVATOR_CONFIG_0.MotionMagic.MotionMagicExpo_kV = 0.12;

            ELEVATOR_CONFIG_1.MotionMagic.MotionMagicCruiseVelocity = 200;
            ELEVATOR_CONFIG_1.MotionMagic.MotionMagicAcceleration = 45;
            ELEVATOR_CONFIG_1.MotionMagic.MotionMagicExpo_kV = 0.12;

            ELEVATOR_CONFIG_0.CurrentLimits.SupplyCurrentLimitEnable = true;
            ELEVATOR_CONFIG_0.CurrentLimits.SupplyCurrentLowerLimit = 30;
            ELEVATOR_CONFIG_0.CurrentLimits.SupplyCurrentLimit = 60;
            ELEVATOR_CONFIG_0.CurrentLimits.SupplyCurrentLowerTime = 1;

            ELEVATOR_CONFIG_1.CurrentLimits.SupplyCurrentLimitEnable = true;
            ELEVATOR_CONFIG_1.CurrentLimits.SupplyCurrentLowerLimit = 30;
            ELEVATOR_CONFIG_1.CurrentLimits.SupplyCurrentLimit = 60;
            ELEVATOR_CONFIG_1.CurrentLimits.SupplyCurrentLowerTime = 1;

        }

        public static TalonFXConfiguration COAST_MODE_CONFIGURATION = new TalonFXConfiguration();
        static {
            COAST_MODE_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            COAST_MODE_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        // TODO: Retune these numbers ~~~
        // Preset Heights
        public static final Distance HEIGHT_CORAL_L1 = Units.Inches.of(3.8);
        public static final Distance HEIGHT_CORAL_L2 = Units.Inches.of(15);
        public static final Distance HEIGHT_CORAL_L3 = Units.Inches.of(30);
        public static final Distance HEIGHT_CORAL_L4 = Units.Inches.of(54);

        public static final Distance HEIGHT_ALGAE_GROUND = Units.Inches.of(0);
        public static final Distance HEIGHT_ALGAE_LOW = Units.Inches.of(23);
        public static final Distance HEIGHT_ALGAE_HIGH = Units.Inches.of(40);

        public static final Distance HEIGHT_NET = Units.Inches.of(54);
        public static final Distance HEIGHT_PROCESSOR = Units.Inches.of(1);

        // Physical Constants
        public static final Distance ELEVATOR_MIN_HEIGHT = Units.Inches.of(0);
        public static final Distance ELEVATOR_MAX_HEIGHT = Units.Inches.of(54);

        public static final Distance DEADZONE_DISTANCE = Units.Inches.of(0.2);
        public static final Distance ZERO_DEADZONE_DISTANCE = Units.Inches.of(0.1);

        public static final Time ELEVATOR_MAX_TIMEOUT = Time.ofBaseUnits(0.4, Seconds);

        // TODO: }] Tune End here~

        public static final AngularVelocity MANUAL_ZEROING_START_VELOCITY = Units.RotationsPerSecond.of(5);
        public static final AngularVelocity MANUAL_ZEROING_DELTA_VELOCITY = Units.RotationsPerSecond.of(5);

        /**
         * Voltage given to motor when it's zeroing
         */
        public static final Voltage ZEROING_VOLTAGE = Units.Volts.of(-4);

        /**
         * Zero position (yk could slide around a bit~~~i pray it's zero)
         */
        public static final Distance ZEROED_POS = Units.Inches.of(0);

        /**
         * The velocity that the motor goes at once it has zeroed (and can no longer
         * continue in that direction)
         */
        public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

        public static final Time ZEROED_TIME = Units.Seconds.of(1.2);

        public static final Time ZEROING_TIMEOUT = Units.Seconds.of(3);
    }

    public static class CONSTANTS_WRIST {
        public static final double WRIST_GEAR_RATIO = 279.27;
        public static final TalonFXConfiguration WRIST_CONFIG = new TalonFXConfiguration();

        public static final Angle MAX_POS = Units.Degrees.of(35);
        public static final Angle MIN_POS = Units.Degrees.of(-80);

        static {
            WRIST_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            WRIST_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_POS.in(Units.Rotations);
            WRIST_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            WRIST_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_POS.in(Units.Rotations);

            WRIST_CONFIG.Feedback.FeedbackRemoteSensorID = CONSTANTS_PORTS.WRIST_ENCODER_CAN;
            WRIST_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            WRIST_CONFIG.Feedback.RotorToSensorRatio = WRIST_GEAR_RATIO;
            // WRIST_CONFIG.Feedback.SensorToMechanismRatio = WRIST_GEAR_RATIO;

            WRIST_CONFIG.Slot0.kG = 0.0; // Volts to overcome gravity
            WRIST_CONFIG.Slot0.kS = 0.3; // Volts to overcome static friction
            WRIST_CONFIG.Slot0.kV = 0.3; // Volts for a velocity target of 1 rps
            WRIST_CONFIG.Slot0.kA = 0.01; // Volts for an acceleration of 1 rps/s
            WRIST_CONFIG.Slot0.kP = 60;
            WRIST_CONFIG.Slot0.kI = 0;
            WRIST_CONFIG.Slot0.kD = 0.8;

            WRIST_CONFIG.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            // WRIST_CONFIG.Slot0.StaticFeedforwardSign =
            // StaticFeedforwardSignValue.UseClosedLoopSign;

            WRIST_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 50;
            WRIST_CONFIG.MotionMagic.MotionMagicAcceleration = 2100;
        }
        public static final Angle PIVOT_INTAKE_CORAL = Units.Degrees.of(-70);
        public static final Angle PIVOT_ALGAE_GROUND = Units.Degrees.of(30);
        public static final Angle PIVOT_ALGAE_REEF = Units.Degrees.of(20);
        public static final Angle PIVOT_SCORE_CORAL = Units.Degrees.of(-68);
        public static final Angle PIVOT_ALGAE_NET = Units.Degrees.of(-60);
        public static final Angle PIVOT_ALGAE_NEUTRAL = Units.Degrees.of(-60);
        public static final Angle PIVOT_CLIMB = Units.Degrees.of(-68);
        public static final Angle PIVOT_DEFAULT = Units.Degrees.of(-80);
        // TODO: add processor scoring angle

        public static final Angle DEADZONE_DISTANCE = Units.Degrees.of(1);
        public static final Time WRIST_TIMEOUT = Time.ofRelativeUnits(0.6, Seconds);
    }

    public static class CONSTANTS_ALGAE {
        public static final double ALGAE_INTAKE_SPEED = 0.4;
        public static final double ALGAE_OUTTAKE_SPEED = -0.5;
        public static final double ALGAE_IDLE_SPEED = 0;
        public static final double ALGAE_HOLD_SPEED = 0.25;

        public static final TalonFXConfiguration ALGAE_INTAKE_CONFIG = new TalonFXConfiguration();

        static {
            ALGAE_INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ALGAE_INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        public static final AngularVelocity ALGAE_INTAKE_OCCUPIED_VELOCITY = Units.RotationsPerSecond.of(0.25);
        public static final Current ALGAE_INTAKE_OCCUPIED_CURRENT = Units.Amps.of(18);
    }

    public static class CONSTANTS_CORAL {
        public static TalonFXConfiguration CORAL_CONFIG = new TalonFXConfiguration();
        static {
            CORAL_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            CORAL_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            Slot0Configs slot0 = CORAL_CONFIG.Slot0;
            slot0.kS = 0;
            slot0.kV = 0.1;
            slot0.kA = 0.01;
            slot0.kP = 8;
            slot0.kI = 0;
            slot0.kD = 0;
        }

        public static CANrangeConfiguration CORAL_SENSOR_CONFIG = new CANrangeConfiguration();

        public static final Distance INDEXED_CORAL_DISTANCE = Units.Inches.of(2);

        public static final Time CORAL_SCORE_TIME = Units.Second.of(0.5);

        public static final double CORAL_OUTTAKE_SPEED = 0.375;

        public static final double CORAL_INTAKE_SPEED = 0.2;
    }

    public static class CONSTANTS_RAMP {
        // 45:1 ratio
        public static final double RAMP_UP_VELOCITY = 1;
        public static final double RAMP_DOWN_VELOCITY = -1;
        public static final double RAMP_INTAKE_VELOCITY = -0.1;

        public static TalonFXConfiguration RAMP_CONFIG = new TalonFXConfiguration();
        // TODO: find the real numbers for these
        public static Angle MAX_POSITION = Units.Rotations.of((45.0 / 360.0) * 49.0);
        public static Angle MIN_POSITION = Units.Rotations.of((45.0 / 360.0) * 0.0);

        public static Angle POSITION_TOLERANCE = Units.Rotations.of(0.1);

        static {
            RAMP_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            RAMP_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;

            RAMP_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            RAMP_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            RAMP_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_POSITION.in(Units.Rotations);
            RAMP_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            RAMP_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_POSITION.in(Units.Rotations);
        }
    }

    public static class CONSTANTS_CLIMB {
        // 75 : 1 Climber ratio
        public static final double CLIMBER_DEPLOYING_VELOCITY = 1;
        public static final double CLIMBER_RETRACT_VELOCITY = -0.8;

        public static TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();
        // TODO: find the real numbers for these
        // The climber is zero generally in the same spot when setup (allow for
        // tolerance)
        public static Angle MAX_POSITION = Units.Rotations.of((75.0 / 360.0) * 80.0);
        public static Angle MIN_POSITION = Units.Rotations.of((75.0 / 360.0) * -30.0);

        public static Angle POSITION_TOLERANCE = Units.Rotations.of(9);

        static {
            CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
            CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimit = 85;
            CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 60;

            CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            CLIMBER_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            CLIMBER_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_POSITION.in(Units.Rotations);
            CLIMBER_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            CLIMBER_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_POSITION.in(Units.Rotations);
        }
    }

    public static class CONSTANTS_VISION {
        // AprilTag layout
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeAndyMark);

        // TODO: change these names on limelight config :))) pls do or it wont work
        public static final String[] LIMELIGHT_NAMES = new String[] { "limelight-front", "limelight-back" };

        /**
         * <p>
         * Pose estimator standard deviation for vision data
         * <p>
         * <b>Units:</b> Meters
         */
        public static final double MEGA_TAG2_STD_DEVS_POSITION = 0.7;

        /**
         * <p>
         * Pose estimator standard deviation for vision data (not actually used XD)
         * </p>
         * <b>Units:</b> Radians
         */
        public static final double MEGA_TAG2_STD_DEVS_HEADING = 9999999;

        /**
         * <p>
         * Pose estimator standard deviation for vision data
         * </p>
         * <b>Units:</b> Meters
         */
        public static final double MEGA_TAG1_STD_DEVS_POSITION = 0.3;

        public static final double MEGA_TAG1_STD_DEVS_HEADING = 0.1;
        /**
         * <p>
         * Maximum rate of rotation before we begin rejecting pose updates
         * </p>
         */
        public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(720);

        /**
         * The area that one tag (if its the only tag in the update) needs to exceed
         * before being accepted
         */
        public static final double AREA_THRESHOLD = 0.05;

        // TODO: get these numbers so it looks clean
        // Position values of our limelights : THIS ISN'T USED IN CODE, ONLY IN THE
        // LIMELIGHT CONFIG
        public static class LIMELIGHT_FRONT {
            public static final Distance LL_FORWARD = Units.Meters.of(0);
            public static final Distance LL_RIGHT = Units.Meters.of(0);
            public static final Distance LL_UP = Units.Meters.of(0);

            public static final Angle LL_ROLL = Units.Degrees.of(0);
            public static final Angle LL_PITCH = Units.Degrees.of(0);
            public static final Angle LL_YAW = Units.Degrees.of(0);
        }

        public static class LIMELIGHT_BACK {
            public static final Distance LL_FORWARD = Units.Meters.of(0);
            public static final Distance LL_RIGHT = Units.Meters.of(0);
            public static final Distance LL_UP = Units.Meters.of(0);

            public static final Angle LL_ROLL = Units.Degrees.of(0);
            public static final Angle LL_PITCH = Units.Degrees.of(0);
            public static final Angle LL_YAW = Units.Degrees.of(0);
        }
    }

    public static class CONSTANTS_USBCAM {
        public static final int CAM01_ID = 0;
        public static final int CAM02_ID = 1;

        public static final int FPS = 15;
        public static final int RES_WIDTH = 640;
        public static final int RES_HEIGHT = 480;

    }

    public static class CONSTANTS_FIELD {
        public static Optional<Alliance> ALLIANCE = Optional.empty();
        public static final Distance FIELD_LENGTH = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
        public static final Distance FIELD_WIDTH = Units.Feet.of(26).plus(Units.Inches.of(5));

        /**
         * Boolean that controls when the path will be mirrored for the red
         * alliance. This will flip the path being followed to the red side of the
         * field.
         * The origin will remain on the Blue side.
         * 
         * @return If we are currently on Red alliance. Will return false if no alliance
         *         is found
         */
        public static boolean isRedAlliance() {
            var alliance = ALLIANCE;

            var x = DriverStation.getAlliance();

            if (x.isPresent() && x.get() == DriverStation.Alliance.Red) {
                return true;
            }
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };

        public static Pose2d getRelativePose(Pose2d reefPose, double xOffset, double yOffset) {
            Translation2d offset = new Translation2d(xOffset, yOffset);
            Translation2d transformedOffset = offset.rotateBy(reefPose.getRotation());
            return new Pose2d(
                    reefPose.getTranslation().plus(transformedOffset),
                    reefPose.getRotation());
        }

        /*
         * All poses on the field, defined by their location on the BLUE Alliance
         */
        public static class POSES {
            public static final Pose2d RESET_POSE = new Pose2d(3.169, 4.015, new Rotation2d());

            public static final Pose3d SCORING_ELEMENT_NOT_COLLECTED = new Pose3d(0, 0, -1, Rotation3d.kZero);

            // BRANCH POSES
            // negative goes away from reef
            public static final double REEF_SCORE_X_OFFSET = -0.14;
            public static final double REEF_SCORE_Y_OFFSET = 0;

            public static final Pose2d REEF_A = getRelativePose(new Pose2d(3.171, 4.189, Rotation2d.fromDegrees(0)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_B = getRelativePose(new Pose2d(3.171, 3.863, Rotation2d.fromDegrees(0)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_C = getRelativePose(new Pose2d(3.688, 2.968, Rotation2d.fromDegrees(60)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_D = getRelativePose(new Pose2d(3.975, 2.803, Rotation2d.fromDegrees(60)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_E = getRelativePose(new Pose2d(5.001, 2.804, Rotation2d.fromDegrees(120)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_F = getRelativePose(new Pose2d(5.285, 2.964, Rotation2d.fromDegrees(120)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_G = getRelativePose(new Pose2d(5.805, 3.863, Rotation2d.fromDegrees(180)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_H = getRelativePose(new Pose2d(5.805, 4.189, Rotation2d.fromDegrees(180)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_I = getRelativePose(new Pose2d(5.288, 5.083, Rotation2d.fromDegrees(-120)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_J = getRelativePose(new Pose2d(5.002, 5.248, Rotation2d.fromDegrees(-120)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);
            public static final Pose2d REEF_K = getRelativePose(new Pose2d(3.972, 5.247, Rotation2d.fromDegrees(-60)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);

            // public static final Pose2d REEF_K = new Pose2d(3.972-0.5,
            // 5.247+.5*Math.sqrt(3), Rotation2d.fromDegrees(-60));

            public static final Pose2d REEF_L = getRelativePose(new Pose2d(3.693, 5.079, Rotation2d.fromDegrees(-60)),
                    REEF_SCORE_X_OFFSET,
                    REEF_SCORE_Y_OFFSET);

            private static final List<Pose2d> BLUE_REEF_POSES = List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
                    REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L);
            private static final List<Pose2d> RED_REEF_POSES = getRedReefPoses();

            // CORAL STATION POSES
            public static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(1.64, 7.33, Rotation2d.fromDegrees(-54.5));
            public static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(-54.5));
            public static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(1.61, 0.70, Rotation2d.fromDegrees(55));
            public static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(55));

            private static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_CORAL_STATION_FAR,
                    LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR);
            private static final List<Pose2d> RED_CORAL_STATION_POSES = getRedCoralStationPoses();

            // processor poses
            public static final Pose2d PROCESSOR = new Pose2d(6, .77, Rotation2d.fromDegrees(-90));

            private static final Pose2d BLUE_PROCESSOR_POSE = PROCESSOR;
            private static final Pose2d RED_PROCESSOR_POSE = getRedProcessorPose();
            private static final List<Pose2d> PROCESSOR_POSES = List.of(BLUE_PROCESSOR_POSE, RED_PROCESSOR_POSE);

            private static final Pose2d[] BLUE_POSES = new Pose2d[] { RESET_POSE, REEF_A, REEF_B, REEF_C, REEF_D,
                    REEF_E,
                    REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L };

            private static final Pose2d[] RED_POSES = getRedAlliancePoses();

        }

        public static Pose2d getRedAlliancePose(Pose2d bluePose) {
            return new Pose2d(FIELD_LENGTH.in(Units.Meters) - (bluePose.getX()),
                    FIELD_WIDTH.in(Units.Meters) - bluePose.getY(),
                    bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
        }

        private static Pose2d[] getRedAlliancePoses() {
            Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_POSES.length];

            for (int i = 0; i < POSES.BLUE_POSES.length; i++) {
                returnedPoses[i] = getRedAlliancePose(POSES.BLUE_POSES[i]);
            }
            return returnedPoses;
        }

        private static List<Pose2d> getRedReefPoses() {
            Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_REEF_POSES.size()];

            for (int i = 0; i < POSES.BLUE_REEF_POSES.size(); i++) {
                returnedPoses[i] = getRedAlliancePose(POSES.BLUE_REEF_POSES.get(i));
            }

            return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
                    returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9],
                    returnedPoses[10],
                    returnedPoses[11]);
        }

        private static List<Pose2d> getRedCoralStationPoses() {
            Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_CORAL_STATION_POSES.size()];

            for (int i = 0; i < POSES.BLUE_CORAL_STATION_POSES.size(); i++) {
                returnedPoses[i] = getRedAlliancePose(POSES.BLUE_CORAL_STATION_POSES.get(i));
            }

            return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3]);
        }

        private static Pose2d getRedProcessorPose() {
            Pose2d returnedPose = POSES.BLUE_PROCESSOR_POSE;

            returnedPose = getRedAlliancePose(POSES.BLUE_PROCESSOR_POSE);

            return returnedPose;
        }

        /**
         * Gets the positions of all of the necessary field elements on the field. All
         * coordinates are in meters and are relative to the blue alliance.
         * 
         * @return An array of field element positions
         */
        public static Supplier<Pose2d[]> getFieldPositions() {
            if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
                return () -> POSES.RED_POSES;

            }
            return () -> POSES.BLUE_POSES;
        }

        /**
         * Gets the positions of all of the necessary field elements on the field. All
         * coordinates are in meters and are relative to the blue alliance.
         * 
         * @return An array of the reef branches for your alliance
         */
        public static Supplier<List<Pose2d>> getReefPositions() {
            if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
                return () -> POSES.RED_REEF_POSES;

            }
            return () -> POSES.BLUE_REEF_POSES;
        }

        public static Supplier<List<Pose2d>> getCoralStationPositions() {
            if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
                return () -> POSES.RED_CORAL_STATION_POSES;
            }
            return () -> POSES.BLUE_CORAL_STATION_POSES;
        }

        public static Supplier<List<Pose2d>> getProcessorPositions() {
            return () -> POSES.PROCESSOR_POSES;
        }
    }

    public static class CONSTANTS_LED {

    }
}