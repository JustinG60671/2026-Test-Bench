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
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Outtake.OuttakeStates;

/**
 * Declares the robot's subsystems, commands, and controller button bindings.
 * In the command-based paradigm, robot logic lives here rather than in
 * {@link Robot}'s periodic methods. The {@link #configureBindings()} method
 * maps controller inputs to commands for both the driver and operator.
 */
public class RobotContainer {
   
    public static final CommandXboxController driver = new CommandXboxController(Constants.Controllers.DRIVER_PORT);

    public static final CommandXboxController operator = new CommandXboxController(Constants.Controllers.OPERATOR_PORT);

    public RobotContainer() {
        configureBindings();
    }
    private void configureBindings() {
        
        //operator controls

        operator.rightBumper().onTrue(
            new InstantCommand(() -> Robot.getOuttake().setOuttake(OuttakeStates.OUT), Robot.getOuttake())
        ).onFalse(
            new InstantCommand(() -> Robot.getOuttake().setOuttake(OuttakeStates.OFF), Robot.getOuttake())
        );

        operator.leftBumper().onTrue(
            new InstantCommand(() -> Robot.getIntake().setIntake(IntakeStates.IN), Robot.getIntake())
        ).onFalse(
            new InstantCommand(() -> Robot.getIntake().setIntake(IntakeStates.OFF), Robot.getIntake())
        );

    }
}
