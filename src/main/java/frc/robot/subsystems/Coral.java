package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

public class Coral extends SubsystemBase {
    TalonFX coralMotor;
    CANrange coralSensor;
    private boolean hasCoral;
    private DigitalInput coralElevatorSensor;

    public Coral() {
        coralMotor = new TalonFX(CONSTANTS_PORTS.CORAL_CAN);
        coralSensor = new CANrange(CONSTANTS_PORTS.CORAL_SENSOR_CAN);
        coralElevatorSensor = new DigitalInput(CONSTANTS_PORTS.CORAL_ELEVATOR_SENSOR_CHANNEL);

        hasCoral = false;

        coralMotor.getConfigurator().apply(CONSTANTS_CORAL.CORAL_CONFIG, 0.25);
        coralSensor.getConfigurator().apply(CONSTANTS_CORAL.CORAL_SENSOR_CONFIG);
    }

    public void setCoralMotor(double speed) {
        coralMotor.set(speed);
    }

    public boolean hasCoral() {
        return coralSensor.getDistance().getValueAsDouble() < 0.1;
    }

    public boolean coralCleared() {
        return !coralElevatorSensor.get();
    }

    public boolean coralLoaded() {
        return (hasCoral() && !coralCleared());
    }

    @Override
    public void periodic() {

    }
}