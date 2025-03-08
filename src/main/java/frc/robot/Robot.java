package frc.robot;

import com.google.flatbuffers.Constants;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CONSTANTS.CONSTANTS_FIELD;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  boolean hasAutonomousRun = false;
  private boolean bothSubsystemsZeroed = false;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // TODO: fix bind
    // Epilogue.bind(this);
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.AddVisionMeasurement().schedule();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setMegaTag2(false);
    if (!hasAutonomousRun) {
      // Manual Zero Command here
    }
    
  }

  @Override
  public void disabledPeriodic() {
    CONSTANTS_FIELD.ALLIANCE = DriverStation.getAlliance();
    SmartDashboard.putString("ALLIANCE", CONSTANTS_FIELD.ALLIANCE.toString());
    if (!hasAutonomousRun) {
      m_robotContainer.resetToAutoPose();
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.setMegaTag2(true);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    hasAutonomousRun = true;
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.setMegaTag2(true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (!hasAutonomousRun || !bothSubsystemsZeroed) {
      m_robotContainer.zeroSubsystems.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }
}