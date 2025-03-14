package frc.robot.subsystems.swerve;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.CONSTANTS_DRIVETRAIN;

public class Swerve extends SubsystemBase {
	public Module[] modules;
	public SwerveDrivePoseEstimator swervePoseEstimator;
	public SwerveDriveKinematics swerveKinematics;
	public Pigeon2 pigeon;
	public boolean isFieldRelative;

	public SwerveConstants swerveConstants;
	public PIDConstants autoDrivePID;
	public PIDConstants autoSteerPID;
	private Matrix<N3, N1> stateStdDevs;
	private Matrix<N3, N1> visionStdDevs;
	public HashMap<String, Command> autoEventMap = new HashMap<>();
	public BooleanSupplier autoFlipPaths;

	public PathPlannerTrajectory exampleAuto;

	private boolean isSimulation;
	public double simAngle = 0;
	private SwerveModuleState[] lastDesiredStates = new SwerveModuleState[] { new SwerveModuleState(),
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
	public double timeFromLastUpdate = 0;
	public double lastSimTime = Timer.getFPGATimestamp();
	public Field2d field;

	/**
	 * @param swerveConstants
	 *                            The constants for all of your modules, such as
	 *                            gear ratio and max
	 *                            speed. You can create your own SN_SwerveConstants
	 *                            object or use a
	 *                            preset.
	 * @param modules
	 *                            An array of SN_SwerveModules.
	 * @param wheelbase
	 *                            Physically measured distance (center to center)
	 *                            between the Left
	 *                            and Right wheels in meters
	 * @param trackWidth
	 *                            Physically measured distance (center to center)
	 *                            between the Front
	 *                            and Back wheels in meters
	 * @param CANBusName
	 *                            The name of the CANBus that all of the swerve
	 *                            components are on
	 * @param pigeonCANId
	 *                            The CAN id of the Pigeon. The Pigeon MUST be on
	 *                            the same CANBus as
	 *                            the modules
	 * @param minimumSteerPercent
	 *                            The minimum PercentOutput required to make the
	 *                            steer motor move
	 * @param driveInversion
	 *                            The direction that is positive for drive motors
	 * @param steerInversion
	 *                            The direction that is positive for steer motors
	 * @param cancoderInversion
	 *                            The direction that is positive for Cancoders
	 * @param driveNeutralMode
	 *                            The behavior of every drive motor when set to
	 *                            neutral-output
	 * @param steerNeutralMode
	 *                            The behavior of every steer motor when set to
	 *                            neutral-output
	 * @param stateStdDevs
	 *                            Standard deviations of the pose estimate (x
	 *                            position in meters, y
	 *                            position in meters, and heading in radians).
	 *                            Increase these
	 *                            numbers to trust your state estimate less.
	 * @param visionStdDevs
	 *                            Standard deviations of vision pose measurements (x
	 *                            position in
	 *                            meters, y position in meters, and heading in
	 *                            radians). Increase
	 *                            these numbers to trust the vision pose measurement
	 *                            less.
	 * @param autoDrivePID
	 *                            The translational PID constants applied to the
	 *                            entire Drivetrain
	 *                            during autonomous in order to reach the correct
	 *                            pose
	 * @param autoSteerPID
	 *                            The rotational PID constants applied to the entire
	 *                            Drivetrain
	 *                            during autonomous in order to reach the correct
	 *                            pose
	 * @param robotConfig
	 *                            The robot configuration used by PathPlanner.
	 * @param autoFlipPaths
	 *                            Determines if paths should be flipped to the other
	 *                            side of the
	 *                            field. This will maintain a global blue alliance
	 *                            origin. Used for
	 *                            PathPlanner
	 * @param isSimulation
	 *                            If your robot is running in Simulation. As of
	 *                            2023, you can supply
	 *                            this with Robot.isSimulation();
	 */
	public Swerve(SwerveConstants swerveConstants, Module[] modules, double wheelbase,
			double trackWidth, String CANBusName, int pigeonCANId, double minimumSteerPercent,
			InvertedValue driveLeftInversion, InvertedValue driveRightInversion, InvertedValue steerInversion,
			SensorDirectionValue cancoderInversion,
			NeutralModeValue driveNeutralMode, NeutralModeValue steerNeutralMode, Matrix<N3, N1> stateStdDevs,
			Matrix<N3, N1> visionStdDevs, PIDConstants autoDrivePID, PIDConstants autoSteerPID, RobotConfig robotConfig,
			BooleanSupplier autoFlipPaths, boolean isSimulation) {

		isFieldRelative = true;
		field = new Field2d();

		// Location of all modules in the WPILib robot coordinate system
		swerveKinematics = new SwerveDriveKinematics(
				new Translation2d(wheelbase / 2.0, trackWidth / 2.0),
				new Translation2d(wheelbase / 2.0, -trackWidth / 2.0),
				new Translation2d(-wheelbase / 2.0, trackWidth / 2.0),
				new Translation2d(-wheelbase / 2.0, -trackWidth / 2.0));

		this.modules = modules;
		this.stateStdDevs = stateStdDevs;
		this.visionStdDevs = visionStdDevs;
		this.swerveConstants = swerveConstants;
		this.autoDrivePID = autoDrivePID;
		this.autoSteerPID = autoSteerPID;
		this.isSimulation = isSimulation;
		this.autoFlipPaths = autoFlipPaths;

		Module.isSimulation = isSimulation;
		Module.wheelCircumference = swerveConstants.wheelCircumference;
		Module.maxModuleSpeedMeters = swerveConstants.maxSpeedMeters;
		Module.driveGearRatio = swerveConstants.driveGearRatio;
		Module.steerGearRatio = swerveConstants.steerGearRatio;

		Module.CANBusName = CANBusName;
		Module.minimumSteerSpeedPercent = minimumSteerPercent;

		Module.driveNeutralMode = driveNeutralMode;

		Module.steerNeutralMode = steerNeutralMode;

		Module.steerInversion = steerInversion;
		Module.cancoderInversion = cancoderInversion;

		pigeon = new Pigeon2(pigeonCANId, CANBusName);

		// The absolute encoders need time to initialize
		Timer.delay(5);
		resetModulesToAbsolute();
		configure();

		AutoBuilder.configure(this::getPose, this::resetPoseToPose,
				this::getChassisSpeeds, this::driveAutonomous,
				new PPHolonomicDriveController(autoDrivePID, autoSteerPID), robotConfig,
				autoFlipPaths, this);
	}

	public Command runPathT(String pathName) {
		try {
			// Load the path you want to follow using its name in the GUI
			PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

			// Create a path following command using AutoBuilder. This will also trigger
			// event markers.
			return AutoBuilder.followPath(path);
		} catch (Exception e) {
			DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
			return Commands.none();
		}
	}

	/*
	 * public Command followPathCommand(String pathName) {
	 * try {
	 * PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
	 * 
	 * return new FollowPathCommand(
	 * path,
	 * this::getPose, // Robot pose supplier
	 * this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
	 * this::driveAutonomousFF, // Method that will drive the robot given ROBOT
	 * RELATIVE ChassisSpeeds, AND
	 * // feedforwards
	 * new PPHolonomicDriveController(
	 * CONSTANTS_DRIVETRAIN.AUTO.AUTO_DRIVE_PID,
	 * CONSTANTS_DRIVETRAIN.AUTO.AUTO_STEER_PID),
	 * CONSTANTS_DRIVETRAIN.AUTO.ROBOT_CONFIG,
	 * () -> {
	 * var alliance = DriverStation.getAlliance();
	 * if (alliance.isPresent()) {
	 * return alliance.get() == DriverStation.Alliance.Red;
	 * }
	 * return false;
	 * },
	 * this);
	 * } catch (Exception e) {
	 * DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
	 * return Commands.none();
	 * }
	 * }
	 */

	public void configure() {
		pigeon.setYaw(0);

		for (Module mod : modules) {
			mod.configure();
		}

		swervePoseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, getGyroRotation(), getModulePositions(),
				new Pose2d(), stateStdDevs, visionStdDevs);

	}

	/**
	 * Reset all of the steer motors's encoders to the absolute encoder values.
	 */
	public void resetModulesToAbsolute() {
		for (Module mod : modules) {
			mod.resetSteerMotorToAbsolute();
		}
	}

	/**
	 * Get the position (distance, angle) of each module.
	 *
	 * @return An Array of Swerve module positions (distance, angle)
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (Module mod : modules) {
			positions[mod.moduleNumber] = mod.getModulePosition();
		}

		return positions;
	}

	/**
	 * Get the actual state (velocity, angle) of each module.
	 *
	 * @return An Array of Swerve module states (velocity, angle)
	 */
	public SwerveModuleState[] getActualModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (Module mod : modules) {
			states[mod.moduleNumber] = mod.getActualModuleState();
		}

		return states;
	}

	/**
	 * Get the last desired states (velocity, angle) of each module.
	 *
	 * @return An Array of Swerve module states (velocity, angle)
	 */
	public SwerveModuleState[] getDesiredModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (Module mod : modules) {
			states[mod.moduleNumber] = mod.getDesiredModuleState();
		}

		return lastDesiredStates;
	}

	/**
	 * Returns the robot-relative chassis speeds.
	 *
	 * @return The robot-relative chassis speeds
	 */
	public ChassisSpeeds getChassisSpeeds() {
		return swerveKinematics.toChassisSpeeds(getActualModuleStates());
	}

	/**
	 * Set the states (velocity and position) of the modules.
	 *
	 * @param desiredModuleStates
	 *                            Desired states to set the modules to
	 * @param isOpenLoop
	 *                            Are the modules being set based on open loop or
	 *                            closed loop
	 *                            control
	 *
	 */
	public void setModuleStates(SwerveModuleState[] desiredModuleStates, boolean isOpenLoop) {
		// Lowers the speeds if needed so that they are actually achievable. This has to
		// be done here because speeds must be lowered relative to the other speeds as
		// well
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, swerveConstants.maxSpeedMeters);
		lastDesiredStates = desiredModuleStates;

		for (int i = 0; i < modules.length; i++) {
			modules[i].setModuleState(desiredModuleStates[i], isOpenLoop, false);
		}
	}

	public void setStatesAuto(SwerveModuleState[] desiredModuleStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, swerveConstants.maxSpeedMeters);

		for (int i = 0; i < modules.length; i++) {
			modules[i].setTargetState(desiredModuleStates[i]);
		}

	}

	/**
	 * Drive the drivetrain
	 *
	 * @param translation
	 *                    Desired translational velocity in meters per second
	 * @param rotation
	 *                    Desired rotational velocity in radians per second
	 * @param isOpenLoop
	 *                    Are the modules being set based on open loop or closed
	 *                    loop
	 *                    control
	 *
	 */
	public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
		ChassisSpeeds chassisSpeeds;

		if (isFieldRelative) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
					getRotation());
		} else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		SwerveModuleState[] desiredModuleStates = swerveKinematics
				.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
		setModuleStates(desiredModuleStates, isOpenLoop);
	}

	/**
	 * Drive the drivetrain in autonomous. Autonomous driving is always closed loop.
	 *
	 * @param chassisSpeeds
	 *                      Desired robot-relative chassis speeds
	 *
	 */
	public void driveAutonomous(ChassisSpeeds chassisSpeeds) {
		ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

		SwerveModuleState[] desiredModuleStates = swerveKinematics.toSwerveModuleStates(targetSpeeds);
		setModuleStates(desiredModuleStates, true);

	}

	public void driveAutonomousFF(ChassisSpeeds chassisSpeeds, DriveFeedforwards ff) {
		ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

		SwerveModuleState[] desiredModuleStates = swerveKinematics.toSwerveModuleStates(targetSpeeds);
		setModuleStates(desiredModuleStates, false);

	}

	/**
	 * Sets all modules to neutral output
	 */
	public void neutralDriveOutputs() {
		for (Module mod : modules) {
			mod.neutralDriveOutput();
		}
	}

	/**
	 * Sets the drive method to use field relative drive controls
	 */
	public void setFieldRelative() {
		isFieldRelative = true;
	}

	/**
	 * Sets the drive method to use robot relative drive controls
	 */
	public void setRobotRelative() {
		isFieldRelative = false;
	}

	/**
	 * Updates the pose estimator with the current robot uptime, the gyro yaw, and
	 * each swerve module position.
	 * <p>
	 * This method MUST be called every loop (or else pose estimator breaks)
	 */
	public void updatePoseEstimator() {
		swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation(), getModulePositions());
	}

	/**
	 * Return the current estimated pose from the pose estimator.
	 *
	 * @return The current estimated pose
	 */
	public Pose2d getPose() {
		return swervePoseEstimator.getEstimatedPosition();
	}

	/**
	 * Return the current rotation of the robot using the pose estimator.
	 *
	 * @return The current estimated rotation
	 */
	public Rotation2d getRotation() {
		return swervePoseEstimator.getEstimatedPosition().getRotation();
	}

	/**
	 * Reset the pose estimator's pose to a given pose.
	 *
	 * @param pose
	 *             The pose you would like to reset the pose estimator to
	 */
	public void resetPoseToPose(Pose2d pose) {
		swervePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
	}

	/**
	 * Get the rotation of the drivetrain using the Pigeon. In simulation, this will
	 * return a simulated value. The rotation will wrap from 0 to 360 degrees.
	 *
	 * @return Rotation of drivetrain. The Rotation2d object will deal with units
	 *         for you as long as you specify your desired unit (ex.
	 *         rotation.getDegrees)
	 */
	public Rotation2d getGyroRotation() {

		if (isSimulation && lastDesiredStates != null) {
			simAngle += swerveKinematics.toChassisSpeeds(lastDesiredStates).omegaRadiansPerSecond *
					timeFromLastUpdate;

			// Wrap to +- 1 rotation
			simAngle = simAngle % (2 * Math.PI);
			// Wrap to 0 -> 1 rotation
			simAngle = (simAngle < 0) ? simAngle + (2 * Math.PI) : simAngle;
			return Rotation2d.fromRadians(simAngle);
		}
		double yaw = pigeon.getYaw().getValueAsDouble() % 360;
		return (yaw < 0) ? Rotation2d.fromDegrees(yaw + 360) : Rotation2d.fromDegrees(yaw);
	}

	/**
	 * @return The current rate of rotation for the Pigeon 2. <b>Units:</b> Degrees
	 *         per Second
	 */
	public double getGyroRate() {
		return pigeon.getAngularVelocityZWorld().getValueAsDouble();
	}

	/**
	 * Resets the Yaw of the Pigeon to the given value
	 *
	 * @param yaw
	 *            The yaw (in degrees) to reset the Pigeon to
	 */
	public void resetYaw(double yaw) {
		if (isSimulation) {
			simAngle = Units.Radians.convertFrom(yaw, Units.Degrees);
		}

		pigeon.setYaw(yaw);
	}

	/**
	 * Adds a vision measurement to the pose estimator. This method does not need to
	 * be called periodically. The superclass of this class (SN_SuperSwerve) already
	 * updates the pose estimator every loop.
	 *
	 * @param estimatedPose
	 *                      The estimated pose measurement generated by vision
	 * @param timestamp
	 *                      The timestamp of that pose estimate (not necessarily the
	 *                      current
	 *                      timestamp)
	 */
	public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
		swervePoseEstimator.addVisionMeasurement(estimatedPose, timestamp);
	}

	/**
	 * <p>
	 * <b>Must be run periodically in order to function properly!</b>
	 * </p>
	 * Updates the values based on the current timestamp of the robot. Mainly
	 * required for simulation and discretize
	 */
	public void updateTimer() {
		timeFromLastUpdate = Timer.getFPGATimestamp() - lastSimTime;
		lastSimTime = Timer.getFPGATimestamp();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("PigeonYaw", getGyroRotation().getDegrees());
		SmartDashboard.putNumber("SwervePoseEstimator/x", swervePoseEstimator.getEstimatedPosition().getX());
		SmartDashboard.putNumber("SwervePoseEstimator/y", swervePoseEstimator.getEstimatedPosition().getY());
		SmartDashboard.putNumber("SwervePoseEstimator/r",
				swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees());

		updateTimer();
		updatePoseEstimator();
	}
}