package frc.robot.commands.zero;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
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
        if (isZero) {
            return true;
        }

        // If the current velocity is low enough to be considered as zeroed
        if (elevator.getMotorVelocity().lt(CONSTANTS_ELEVATOR.ZEROED_VELOCITY)) {
            // And this is the first loop it has happened, begin the timer
            if (zeroingTimestamp.equals(Units.Seconds.zero())) {
                zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
                return false;
            }

            // If this isn't the first loop, return if it has been below the threshold for
            // long enough
            return (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp)
                    .gte(CONSTANTS_ELEVATOR.ZEROED_TIME));
        }

        // If the above wasn't true, we have gained too much velocity, so we aren't at 0
        // & need to restart the timer
        zeroingTimestamp = Units.Seconds.zero();
        return false;
    }
}