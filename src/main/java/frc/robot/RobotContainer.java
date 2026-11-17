package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.MoveForward;
//import frc.robot.generated.TunerConstants;
//import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Outtake.OuttakeStates;

public class RobotContainer {

    public static final CommandXboxController driver = new CommandXboxController(Constants.Controllers.DRIVER_PORT);
    public static final CommandXboxController operator = new CommandXboxController(Constants.Controllers.OPERATOR_PORT);

    // Swerve limits
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Swerve requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandSwerveDrivetrain drivetrain;

    public RobotContainer() {
        drivetrain = Robot.getDrivetrain();
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                     .withVelocityY(-driver.getLeftX() * MaxSpeed)
                     .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        // Keep the swerve idle (neutral) while disabled
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // A: lock wheels in an X
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // B: point wheels in the direction of the left stick
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Left bumper: reset field-centric heading
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Operator controls

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

    public Command getAutonomousCommand() {
        return new MoveForward();
    }
}