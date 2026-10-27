package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.*;

import com.fasterxml.jackson.databind.deser.impl.NullsAsEmptyProvider;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;
//import frc.robot.vision.commands.TowerAlign;

//import choreo.Choreo;
//import choreo.auto.AutoChooser;
//import choreo.auto.AutoFactory;
//import choreo.auto.AutoRoutine;
//import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
    public static final CommandXboxController driver = new CommandXboxController(Constants.Controllers.DRIVER_PORT);
    public static final CommandXboxController operator = new CommandXboxController(Constants.Controllers.OPERATOR_PORT);

    //public final Rumble rumble = new Rumble(); --not needed for now

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    public RobotContainer() {
        
    }

    private void configureBindings() {
        //shift1WarningTrigger.onTrue(rumble);
        //...
        //engameWarningTrigger.onTrue(rumble);

        Robot.getDrivetrain().setDefaultCommand();
    }
}
