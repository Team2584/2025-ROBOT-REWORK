package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_CORAL;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

public class Coral extends SubsystemBase {
    TalonFX coralMotor;
    CANrange coralSensor;
    private boolean hasCoral;
    private boolean indexingCoral;

    public Coral() {
        coralMotor = new TalonFX(CONSTANTS_PORTS.CORAL_CAN);
        coralSensor = new CANrange(CONSTANTS_PORTS.CORAL_SENSOR_CAN);

        hasCoral = false;

        coralMotor.getConfigurator().apply(CONSTANTS_CORAL.CORAL_CONFIG, 0.25);
        coralSensor.getConfigurator().apply(CONSTANTS_CORAL.CORAL_SENSOR_CONFIG);
    }

    public void setCoralOuttake(double speed) {
        coralMotor.set(speed);
    }

    public void setIndexingCoral(boolean indexing) {
        this.indexingCoral = indexing;
    }

    public boolean isIndexingCoral() {
        return indexingCoral;
    }

    public void setHasCoral(boolean hasCoral) {
        this.hasCoral = hasCoral;
    }

    public void coralToggle() {
        this.hasCoral = !hasCoral;
    }

    public boolean sensorSeesCoral() {
        return coralSensor.getIsDetected().getValue();
    }

    public BooleanSupplier sensorSeesCoralSupplier() {
        return () -> coralSensor.getIsDetected().getValue();
    }

    public boolean sensorIndexedCoral() {
        return coralSensor.getDistance().getValue().gte(CONSTANTS_CORAL.INDEXED_CORAL_DISTANCE);
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}