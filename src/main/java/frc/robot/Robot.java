package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  // systems
  private static CommandSwerveDrivetrain drivetrain;
  private static RobotContainer robotContainer;
  private static Intake intake;
  private static Outtake outtake;

  public Robot() {
    drivetrain = TunerConstants.createDrivetrain();
    robotContainer = new RobotContainer();

    intake = new Intake();
    outtake = new Outtake();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = getRobotContainer().getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  /** Find subsystems */
  public static CommandSwerveDrivetrain getDrivetrain() {
    if (drivetrain == null) {
      drivetrain = TunerConstants.createDrivetrain();
    }
    return drivetrain;
  }

  public static RobotContainer getRobotContainer() {
    if (robotContainer == null) {
      robotContainer = new RobotContainer();
    }
    return robotContainer;
  }

  public static Outtake getOuttake() {
    if (outtake == null) {
      outtake = new Outtake();
    }
    return outtake;
  }

  public static Intake getIntake() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }
}