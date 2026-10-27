// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.commands.Rumble;
// import frc.robot.commands.ShootOnly;
// import frc.robot.commands.AutoClimb;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.Hood.HoodStates;
// import frc.robot.subsystems.Indexer.IndexerStates;
// import frc.robot.vision.commands.TowerAlign;
// import frc.robot.subsystems.Intake.IntakeStates;
// import frc.robot.subsystems.Shooter.ShooterStates;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.*;

import com.fasterxml.jackson.databind.deser.impl.NullsAsEmptyProvider;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Declares the robot's subsystems, commands, and controller button bindings.
 * In the command-based paradigm, robot logic lives here rather than in
 * {@link Robot}'s periodic methods. The {@link #configureBindings()} method
 * maps controller inputs to commands for both the driver and operator.
 */
public class RobotContainer {
   
    public static final CommandXboxController driver = new CommandXboxController(Constants.Controllers.DRIVER_PORT);

    public static final CommandXboxController operator = new CommandXboxController(Constants.Controllers.OPERATOR_PORT);

    
    AutoChooser.addCmd("CrossStart", this::crossStart);

    // private final AutoFactory driveAutoFactory;
    // private final AutoChooser autoChooser;
    private final SendableChooser<Command> autoChooser;
    // autoChooser.addCmd("CrossStart", this::crossStart);

    /** Dashboard chooser for selecting which autonomous routine to run. */
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    /**
     * Constructs the RobotContainer and configures all controller bindings.
     */
    public RobotContainer() {
    

        // NamedCommands.registerCommand("ShooterON", new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.HUB)));
        // NamedCommands.registerCommand("Intake", new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.IN)));
        // NamedCommands.registerCommand("IntakeOFF", new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OFF)));
        // NamedCommands.registerCommand("ShooterOFF", new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.OFF)));
        // NamedCommands.registerCommand("TowerAlign", new TowerAlign(Robot.getDrivetrain()));
        // NamedCommands.registerCommand("Auto Shooter ON", new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.AUTO)));
        // NamedCommands.registerCommand("Auto Hood ON", new InstantCommand(() -> Robot.getHood().setState(HoodStates.AUTO)));
        // NamedCommands.registerCommand("Auto Shooter OFF", new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.OFF)));
        // NamedCommands.registerCommand("Auto Hood OFF", new InstantCommand(() -> Robot.getHood().setState(HoodStates.STOWED)));
        // NamedCommands.registerCommand("Shoot On",  new ParallelCommandGroup(
        //         new InstantCommand(() -> Robot.getIndexer().setState(IndexerStates.IN), Robot.getIndexer()),
        //         new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.IN), Robot.getIntake())));
        // NamedCommands.registerCommand("Shoot OFF", new ParallelCommandGroup(
        //         new InstantCommand(() -> Robot.getIndexer().setState(IndexerStates.OFF), Robot.getIndexer()),
        //         new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OFF), Robot.getIntake())));
        // NamedCommands.registerCommand("Auto Climb", new AutoClimb());


        // autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser.addOption("Auto Climb", new AutoClimb());
        // autoChooser.addOption("ShootOnly", new ShootOnly());
        SmartDashboard.putData("autoChoose", autoChooser);

        configureBindings();
        
    }

    /**
     * Maps controller inputs to commands for both the driver and operator controllers.
     *
     * <p><b>Driver bindings:</b>
     * <ul>
     *   <li>Left stick — strafe (x/y translation)</li>
     *   <li>Right stick X — rotation</li>
     *   <li>Left bumper (held) — switch to robot-relative driving</li>
     *   <li>A button (held) — enable Limelight hub aiming</li>
     *   <li>Right bumper (held) — slow mode</li>
     *   <li>X button — toggle 180-degree gyro correction mode (on press = 0 degrees, on release = 180 degrees)</li>
     *   <li>POV up/down — drive forward/backward with gyro straight correction</li>
     *   <li>POV left/right — strafe left/right with gyro straight correction</li>
     *   <li>Y button (held) — run full tower alignment sequence</li>
     *   <li>Right trigger — run intake in and indexer out</li>
     *   <li>Left trigger — run intake out and indexer in</li>
     * </ul>
     *
     * <p><b>Operator bindings:</b>
     * <ul>
     *   <li>A — turn off shooter, stow hood, stop intake</li>
     *   <li>X — hub shot preset (shooter, hood, intake)</li>
     *   <li>Y — tower shot preset (shooter, hood, intake)</li>
     *   <li>B — feeding preset (shooter, hood, intake)</li>
     *   <li>Right trigger (held) — run indexer and intake in to feed a game piece to the shooter</li>
     * </ul>
     */
    private void configureBindings() {
        Robot.getDrivetrain().setDefaultCommand(
            new RunCommand(
                () -> {
                    // Apply deadband to each axis independently
                    double xSpeed = (Math.abs(driver.getLeftX() * 0.7) > Constants.Controllers.DEADBAND)
                            ? driver.getLeftX() : 0.0;
                    double ySpeed = (Math.abs(driver.getLeftY()) * 0.7 > Constants.Controllers.DEADBAND)
                            ? driver.getLeftY() : 0.0;
                    double rotSpeed = (Math.abs(driver.getRightX()) > Constants.Controllers.DEADBAND)
                        ? driver.getRightX()
                        : 0.0;

                Robot.getDrivetrain().drive(ySpeed, xSpeed, rotSpeed, !driver.getHID().getLeftBumperButton(), driver.getHID().getRightBumperButton());                
            }, Robot.getDrivetrain()
            )
        );

        // X button — toggle gyro correction target heading between 0 and 180 degrees
        driver.x().onTrue(
            new InstantCommand(() -> Robot.getDrivetrain().set180State(false), Robot.getDrivetrain())
        ).onFalse(
            new InstantCommand(() -> Robot.getDrivetrain().set180State(true), Robot.getDrivetrain())
        );

        // POV up — drive forward slowly with gyro straight correction
        driver.povUp().whileTrue(
            new RunCommand(() -> Robot.getDrivetrain().adjustXStraightDrive(-0.11), Robot.getDrivetrain())
        ).onFalse(
            new InstantCommand(() -> Robot.getDrivetrain().drive(0, 0, 0, false, false), Robot.getDrivetrain())
        );

        // POV down — drive backward slowly with gyro straight correction
        driver.povDown().whileTrue(
            new RunCommand(() -> Robot.getDrivetrain().adjustXStraightDrive(0.11), Robot.getDrivetrain())
        ).onFalse(
            new InstantCommand(() -> Robot.getDrivetrain().drive(0, 0, 0, false, false), Robot.getDrivetrain())
        );

        // POV left — strafe left slowly with gyro straight correction
        driver.povLeft().whileTrue(
            new RunCommand(() -> Robot.getDrivetrain().adjustYStraightDrive(-0.11), Robot.getDrivetrain())
        ).onFalse(
            new InstantCommand(() -> Robot.getDrivetrain().drive(0, 0, 0, false, false), Robot.getDrivetrain())
        );

        // POV right — strafe right slowly with gyro straight correction
        driver.povRight().whileTrue(
            new RunCommand(() -> Robot.getDrivetrain().adjustYStraightDrive(0.11), Robot.getDrivetrain())
        ).onFalse(
            new InstantCommand(() -> Robot.getDrivetrain().drive(0, 0, 0, false, false), Robot.getDrivetrain())
        );

    }
    
    /**
     * Checks whether the intake drive nerf should be applied
     * @return is the absolute value of the Right Trigger Axis value greater than our Deadband value
     */
    public boolean intakeNerf() {
        return Math.abs(driver.getHID().getRightTriggerAxis()) > Constants.Controllers.DEADBAND;
    } 

    /**
     * Returns the autonomous command to run during autonomous mode.
     * Currently returns {@code null}, meaning no autonomous routine is configured.
     *
     * @return The autonomous {@link Command} to schedule, or {@code null} for none.
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
        // return autoChooser.selectedCommand();
        // return new Test();

    }

    /**
     * Returns the driver's Xbox controller.
     *
     * @return The driver {@link CommandXboxController} instance.
     */
    public static CommandXboxController getDriver() {
        return driver;
    }

    /**
     * Returns the operator's Xbox controller.
     *
     * @return The operator {@link CommandXboxController} instance.
     */
    public static CommandXboxController getOperator() {
        return operator;
    }

    /**
     * Checks if the current match time is within a threshold of the given time.
     *
     * @param time the target time in seconds
     * @return true if the match time is within 0.1 seconds of the target time
     */
    public boolean withinThresholdTime(double time) {
        if (Math.abs(DriverStation.getMatchTime() - time) < 0.1) {
            return true;
        }
        return false;
    }
}
