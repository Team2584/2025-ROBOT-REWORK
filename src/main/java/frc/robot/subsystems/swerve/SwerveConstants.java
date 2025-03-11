package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(2) * Math.PI;
    public static final double STEER_GEAR_RATIO = 13.371428571428572;
    public double steerGearRatio = 13.371428571428572;
    public double wheelCircumference = Units.inchesToMeters(2) * Math.PI;
    public double driveGearRatio = 5.6;
    public double maxSpeedMeters = 5.33;

    /**
     * A wrapper for Swerve Module constants.
     *
     * @param steerGearRatio
     *                           The gear ratio between the wheel and the steer
     *                           motor
     * @param wheelCircumference
     *                           The circumference of the wheel, in meters
     * @param driveGearRatio
     *                           The gear ratio between the wheel and the drive
     *                           motor
     * @param maxSpeedMeters
     *                           The maximum speed of the module, in meters per
     *                           second
     */
    public SwerveConstants(double steerGearRatio, double wheelCircumference, double driveGearRatio,
            double maxSpeedMeters) {
        this.steerGearRatio = steerGearRatio;
        this.wheelCircumference = wheelCircumference;
        this.driveGearRatio = driveGearRatio;
        this.maxSpeedMeters = maxSpeedMeters;
    }

    public static class KRAKEN {
        /**
         * Preset Constants for a 2584 module's for 2025
         */

        // public static final SwerveConstants L1 = new
        // SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
        // 5.6, Units.feetToMeters(17.5));
    }
}