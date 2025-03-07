package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;
import frc.robot.CONSTANTS.CONSTANTS_USBCAM;

public class Coral extends SubsystemBase {
    TalonFX coralMotor;
    CANrange coralSensor;
    DigitalInput coralElevatorSensor;

    public Coral() {
        coralMotor = new TalonFX(CONSTANTS_PORTS.CORAL_CAN);
        coralSensor = new CANrange(CONSTANTS_PORTS.CORAL_SENSOR_CAN);
        coralElevatorSensor = new DigitalInput(CONSTANTS_PORTS.CORAL_ELEVATOR_SENSOR_CHANNEL);

        coralMotor.getConfigurator().apply(CONSTANTS_CORAL.CORAL_CONFIG, 0.25);
        coralSensor.getConfigurator().apply(CONSTANTS_CORAL.CORAL_SENSOR_CONFIG,0.25);
    }

    public Command intakeCoral() {
        return runEnd(() -> setCoralMotor(CONSTANTS_CORAL.CORAL_INTAKE_SPEED), () -> setCoralMotor(0)).until(() -> coralLoaded());
    }
    
    public Command outtakeCoral() {
        return runEnd(() -> setCoralMotor(CONSTANTS_CORAL.CORAL_OUTTAKE_SPEED), () -> setCoralMotor(0)).withTimeout(2.5);
    }


    public void setCoralMotor(double speed) {
        coralMotor.set(speed);
    }

    public boolean hasCoral() {
        return (coralSensor.getDistance().getValueAsDouble() < 0.1);
    }

    public boolean coralCleared() {
        return !coralElevatorSensor.get();
    }

    public boolean coralLoaded() {
        return (hasCoral() && !coralCleared());
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral/Cleared", coralCleared());
        SmartDashboard.putBoolean("Coral/Loaded", coralLoaded());
        

    }
}