package frc.robot.commands.zero;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.subsystems.Elevator;

public class Zero_Elevator extends Command {
    Elevator elevator;

    Time zeroingTimestamp;
    boolean isZero = false;

    public Zero_Elevator(RobotContainer RC) {
        this.elevator = RC.getElevator();

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setHardLimits(false, true);

        elevator.setVoltage(Units.Volts.zero());
        zeroingTimestamp = Units.Seconds.zero();
        isZero = elevator.isZero;
    }

    @Override
    public void execute() {
        elevator.setVoltage(CONSTANTS_ELEVATOR.ZEROING_VOLTAGE);

        if (elevator.getZeroLimit()) {
            isZero = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setHardLimits(true, true);

        elevator.setVoltage(Units.Volts.zero());

        if (!interrupted) {
            elevator.resetSensorPosition(CONSTANTS_ELEVATOR.ZEROED_POS);
            elevator.isZero = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (elevator.getZeroLimit()) {
            return true;
        }
        return false;
    }
}