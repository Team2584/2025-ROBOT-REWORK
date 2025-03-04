package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    private static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(3.8) * Math.PI;
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    public double steerGearRatio;
    public double wheelCircumference;
    public double driveGearRatio;
    public double maxSpeedMeters;

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
        public static final SwerveConstants L1 = new SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                5.6, Units.feetToMeters(17.5));
    }
}