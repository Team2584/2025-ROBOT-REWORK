package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_ELEVATOR;
import frc.robot.CONSTANTS.CONSTANTS_PORTS;

@Logged
public class Elevator extends SubsystemBase {
    private TalonFX m_Follower_Left;
    private TalonFX m_Leader_Right;

    private Distance lastDesiredPosition;

    Distance currentLeftPosition = Units.Inches.of(0);
    Distance currentRightPosition = Units.Inches.of(0);

    PositionVoltage positionRequest;
    VoltageOut voltageRequest = new VoltageOut(0);

    public boolean tryingZero = false;
    public boolean isZero = false;

    MotionMagicVoltage motionRequest;

    public Elevator() {
        this.m_Leader_Right = new TalonFX(CONSTANTS_PORTS.ELEVATOR_RIGHT_CAN);
        this.m_Follower_Left = new TalonFX(CONSTANTS_PORTS.ELEVATOR_LEFT_CAN);

        lastDesiredPosition = Units.Inches.of(0);
        voltageRequest = new VoltageOut(0);
        motionRequest = new MotionMagicVoltage(0);

        m_Leader_Right.getConfigurator().apply(CONSTANTS_ELEVATOR.ELEVATOR_CONFIG);

        m_Follower_Left.setControl(new Follower(CONSTANTS_PORTS.ELEVATOR_RIGHT_CAN, true));
    }

}
