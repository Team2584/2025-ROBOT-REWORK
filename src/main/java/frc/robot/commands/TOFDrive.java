package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;
import com.ctre.phoenix6.hardware.CANrange;

import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TOFDrive extends Command {
    private final Drivetrain drivetrain;
    private final CANrange TOFDriveSensor;
    private final double distance;
    private final double speed;

    public TOFDrive(RobotContainer RC, double speed, double distance) {
        this.drivetrain = RC.getDrivetrain();
        this.distance = distance;
        this.speed = speed;
        

        this.TOFDriveSensor = new CANrange(CONSTANTS_PORTS.TOF_CAN);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(new ChassisSpeeds(speed, 0, 0),true); // Move forward at set speed
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0),true); // Stop movement
    }

    @Override
    public boolean isFinished() {
        return TOFDriveSensor.getDistance().getValueAsDouble() < distance;
    }
}