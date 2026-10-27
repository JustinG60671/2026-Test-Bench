package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.opencv.core.Mat;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Swerve;
import frc.robot.vision.Limelight;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the swerve drive.
 * Manages four {@link SwerveModule} instances, a Pigeon2 gyro, and odometry
 * for tracking the robot's position on the field. Also handles
 * Limelight-assisted
 * aiming and gyro-based straight-drive correction.
 */
public class SwerveSubsystem extends SubsystemBase {

    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;

    private SlewRateLimiter magLimiter = new SlewRateLimiter(1.8);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(2.0);
    private double prevTime = WPIUtilJNI.now() * 1e-6;

    // Create SwerveModules
    /** Front-left swerve module. */
    private final SwerveModule frontLeftModule = new SwerveModule(
            FRONT_LEFT_DRIVING_CAN_ID,
            FRONT_LEFT_TURNING_CAN_ID,
            FRONT_LEFT_CANCODER,
            CANBUS_NAME,
            FRONT_LEFT_MAGNET_OFFSET,
            FRONT_LEFT_DRIVE_INVERTED);

    /** Front-right swerve module. */
    private final SwerveModule frontRightModule = new SwerveModule(
            FRONT_RIGHT_DRIVING_CAN_ID,
            FRONT_RIGHT_TURNING_CAN_ID,
            FRONT_RIGHT_CANCODER,
            CANBUS_NAME,
            FRONT_RIGHT_MAGNET_OFFSET,
            FRONT_RIGHT_DRIVE_INVERTED);

    /** Back-left swerve module. */
    private final SwerveModule backLeftModule = new SwerveModule(
            BACK_LEFT_DRIVING_CAN_ID,
            BACK_LEFT_TURNING_CAN_ID,
            BACK_LEFT_CANCODER,
            CANBUS_NAME,
            BACK_LEFT_MAGNET_OFFSET,
            BACK_LEFT_DRIVE_INVERTED);

    /** Back-right swerve module. */
    private final SwerveModule backRightModule = new SwerveModule(
            BACK_RIGHT_DRIVING_CAN_ID,
            BACK_RIGHT_TURNING_CAN_ID,
            BACK_RIGHT_CANCODER,
            CANBUS_NAME,
            BACK_RIGHT_MAGNET_OFFSET,
            BACK_RIGHT_DRIVE_INVERTED);

    /** Pigeon2 IMU used for field-relative driving and rotation correction. */
    public final Pigeon2 gyro;

    /**
     * PID controller for x-axis translation correction, used by Choreo autonomous.
     */
    public PIDController pidX = new PIDController(2.0, 0, 0);
    
    /**
     * PID controller for y-axis translation correction, used by Choreo autonomous.
     */
    public PIDController pidY = new PIDController(2.0, 0, 0);
    
    /**
     * PIDController for rotation (used by Choreo).
     */
    public PIDController pidRot = new PIDController(0.5, 0, 0);

    /**
     * Odometry object for tracking the robot's position and heading on the field.
     */
    SwerveDriveOdometry odometry;

    /**
     * Tracks whether the robot's target heading is 180 degrees.
     * Used to switch between 0-degree and 180-degree gyro correction modes
     * during straight-drive alignment.
     */
    public boolean state180 = true;
    


    /**
     * Constructs the SwerveSubsystem. Initializes the CAN bus, gyro, and odometry.
     * Odometry is seeded with the current gyro yaw and all four module positions.
     */
    public SwerveSubsystem() {
        /** CAN bus used for communication with the swerve drive motors and encoders. */
        CANBus canbus = new CANBus(Constants.Swerve.CANBUS_NAME);

        gyro = new Pigeon2(PIGEON_CAN_ID, canbus);

        // Initialize odometry with the current gyro angle and module positions
        odometry = new SwerveDriveOdometry(
            Swerve.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        });

        pidRot.enableContinuousInput(-Math.PI, Math.PI);

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> choreoDrive(speeds, true), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(0.051, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.03, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // // Boolean supplier that controls when the path will be mirrored for the red alliance
                // // This will flip the path being followed to the red side of the field.
                // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                // var alliance = DriverStation.getAlliance();
                // if (alliance.isPresent()) {
                //     return alliance.get() == DriverStation.Alliance.Red;
                // }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    /**
     * Allows the robot to follow a trajectory based on PIDControllers for X, Y, and
     * Rotation.
     * Calculates a ChassisSpeeds object based on these controllers and then runs
     * {@link choreoDrive(ChassisSpeeds)}
     * 
     * @param SwerveSample Object returned by Choreo AutoFactory
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + pidX.calculate(pose.getX(), sample.x),
                sample.vy + pidY.calculate(pose.getY(), sample.y),
                sample.omega + pidX.calculate(pose.getRotation().getRadians(), sample.heading));

        choreoDrive(speeds, true);
    }

    /**
     * Runs every robot loop iteration. Updates odometry with the latest gyro angle
     * and module positions, then publishes telemetry to SmartDashboard.
     */
    @Override
    public void periodic() {
        // Update odometry with the latest gyro and module position readings
        odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            });
            SmartDashboard.putNumber("Odometry in meters X", odometry.getPoseMeters().getX());
            SmartDashboard.putNumber("Odometry in meters Y", odometry.getPoseMeters().getY());
            SmartDashboard.putNumber("front Left Module", frontLeftModule.getPosition().distanceMeters);
            SmartDashboard.putNumber("front Right Module", frontRightModule.getPosition().distanceMeters);
            SmartDashboard.putNumber("back Left Module", backLeftModule.getPosition().distanceMeters);
            SmartDashboard.putNumber("back Right Module", backRightModule.getPosition().distanceMeters);


        SmartDashboard.putNumber("Limelight Distance", Robot.getLimelight().getDistanceFromAprilTagFeet());
        SmartDashboard.putBoolean("TY is Detected", Robot.getLimelight().getTYIsDetected());
        SmartDashboard.putNumber("Gyro Angle", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("FL Encoder Rotation", frontLeftModule.turnEncoderValue);
        SmartDashboard.putNumber("FR Encoder Rotation", frontRightModule.turnEncoderValue);
        SmartDashboard.putNumber("BL Encoder Rotation", backLeftModule.turnEncoderValue);
        SmartDashboard.putNumber("BR Encoder Rotation", backRightModule.turnEncoderValue);
        SmartDashboard.putNumber("FL Drive Encoder Velocity", frontLeftModule.getVelocity());
        SmartDashboard.putNumber("FR Drive Encoder Velocity", frontRightModule.getVelocity());
        SmartDashboard.putNumber("BL Drive Encoder Velocity", backLeftModule.getVelocity());
        SmartDashboard.putNumber("BR Drive Encoder Velocity", backRightModule.getVelocity());



        // SmartDashboard.putNumber("FL Desired State Velocity", frontLeftModule.desiredState.speedMetersPerSecond);
        // SmartDashboard.putNumber("FR Desired State Velocity", frontRightModule.desiredState.speedMetersPerSecond);
        // SmartDashboard.putNumber("BL Desired State Velocity", backLeftModule.desiredState.speedMetersPerSecond);
        // SmartDashboard.putNumber("BR Desired State Velocity", backRightModule.desiredState.speedMetersPerSecond);

        // SmartDashboard.putNumber("FL Desired State Angle", frontLeftModule.desiredState.angle.getDegrees());
        // SmartDashboard.putNumber("FR Desired State Angle", frontRightModule.desiredState.angle.getDegrees());
        // SmartDashboard.putNumber("BL Desired State Angle", backLeftModule.desiredState.angle.getDegrees());
        // SmartDashboard.putNumber("BR Desired State Angle", backRightModule.desiredState.angle.getDegrees());
        // SmartDashboard.putNumber("FR Desired State Angle", frontRightModule.desiredState.angle.getDegrees());
        // SmartDashboard.putNumber("BL Desired State Speed", backLeftModule.desiredState.speedMetersPerSecond);
        // SmartDashboard.putNumber("BR Desired State Speed", backRightModule.desiredState.speedMetersPerSecond);
    }

    /**
     * Returns the current estimated pose of the robot on the field.
     *
     * @return The robot's current {@link Pose2d} from odometry.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets odometry to a given pose. Updates the internal position estimate
     * using the current gyro angle and module positions.
     *
     * @param pose The {@link Pose2d} to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(),
                        backRightModule.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param hubAiming     Whether the robot is currently aiming at the hub using
     *                      the Limelight
     * @param slowIntake    Whether to apply the slow nerf to the maximum speed.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean hubAiming,
            boolean slowIntake, boolean slowDrive) {
        // Convert the commanded speeds into the correct units for the drivetrain
        // Convert the commanded speeds into the correct units for the drivetrain
        double rotDelivered;
        double xSpeedDelivered;
        double ySpeedDelivered;

        if(slowIntake){
            xSpeedDelivered = xSpeed * Swerve.MAX_SPEED_METERS_PER_SECOND * Constants.Swerve.SLOW_INTAKE_NERF;
            ySpeedDelivered = ySpeed * Swerve.MAX_SPEED_METERS_PER_SECOND * Constants.Swerve.SLOW_INTAKE_NERF;
            rotDelivered = rot * Swerve.MAX_ANGULAR_SPEED * Constants.Swerve.SLOW_INTAKE_NERF;
        }else if (slowDrive) {
            xSpeedDelivered = xSpeed * Swerve.MAX_SPEED_METERS_PER_SECOND * Constants.Swerve.SLOW_DRIVE_NERF;
            ySpeedDelivered = ySpeed * Swerve.MAX_SPEED_METERS_PER_SECOND * Constants.Swerve.SLOW_DRIVE_NERF;
            rotDelivered = rot * Swerve.MAX_ANGULAR_SPEED * Constants.Swerve.SLOW_DRIVE_NERF;
        } else {
            xSpeedDelivered = xSpeed * Swerve.MAX_SPEED_METERS_PER_SECOND;
            ySpeedDelivered = ySpeed * Swerve.MAX_SPEED_METERS_PER_SECOND;
            rotDelivered = rot * Swerve.MAX_ANGULAR_SPEED;
        }

        // Override rotation with Limelight aiming if hub aiming is active
        if (hubAiming) {
            Robot.getLimelight().setPipeline(0);
            rotDelivered = limelightAimRotate();
            fieldRelative = false;
        }

        // Convert chassis speeds to individual module states and desaturate to max
        // speed
        var swerveModuleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.MAX_SPEED_METERS_PER_SECOND);

        // Send the target states to each module
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }

    

    /**
     * Method to drive the robot using calculated ChassisSpeeds object.
     * 
     * @param chassisSpeeds Chassis WPILIB Object representing the desired module
     *                      states.
     */
    public void choreoDrive(ChassisSpeeds chassisSpeeds, boolean limited){
        double xSpeed = -chassisSpeeds.vxMetersPerSecond;
        double ySpeed = -chassisSpeeds.vyMetersPerSecond;
        double rotSpeed = -chassisSpeeds.omegaRadiansPerSecond;

        var swerveModuleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
        
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Swerve.MAX_SPEED_METERS_PER_SECOND);

        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }
    // double xSpeed = chassisSpeeds.vxMetersPerSecond / MAX_SPEED_METERS_PER_SECOND;
    //     double ySpeed = chassisSpeeds.vyMetersPerSecond / MAX_SPEED_METERS_PER_SECOND;
    //     double rot = chassisSpeeds.omegaRadiansPerSecond / MAX_ANGULAR_SPEED;

    //     double xSpeedCommanded;
    //     double ySpeedCommanded;
        // if (limited) {
        //     // Convert XY to polar for rate limiting
        //     double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        //     double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

        //     // Calculate the direction slew rate based on an estimate of the lateral acceleration
        //     double directionSlewRate;
        //     if (currentTranslationMag != 0.0) {
        //         directionSlewRate = Math.abs(1.2 / currentTranslationMag);
        //     } else {
        //         directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
        //     }
            
        //     double currentTime = WPIUtilJNI.now() * 1e-6;
        //     double elapsedTime = currentTime - prevTime;
        //     double angleDif = angleDifference(inputTranslationDir, currentTranslationDir);
        //     if (angleDif < 0.45*Math.PI){
        //         currentTranslationDir = stepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        //         currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        //     } else if (angleDif > 0.85*Math.PI) {
        //         if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
        //             // keep currentTranslationDir unchanged
        //             currentTranslationMag = magLimiter.calculate(0.0);
        //         } else {
        //             currentTranslationDir = wrapAngle(currentTranslationDir + Math.PI);
        //             currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        //         }
        //         } else {
        //             currentTranslationDir = stepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        //             currentTranslationMag = magLimiter.calculate(0.0);
        //         }
        //         prevTime = currentTime;
        //         xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
        //         ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
        //         currentRotation = rotLimiter.calculate(rot);


        // } else {
            // xSpeedCommanded = xSpeed;
            // ySpeedCommanded = ySpeed;
            // currentRotation = rot;
        // }
        

        // Convert the commanded speeds into the correct units for the drivetrain
        // double xSpeedDelivered = xSpeedCommanded * Swerve.MAX_SPEED_METERS_PER_SECOND;
        // double ySpeedDelivered = ySpeedCommanded * Swerve.MAX_SPEED_METERS_PER_SECOND;
        // double rotDelivered = currentRotation * Swerve.MAX_ANGULAR_SPEED;

        // var swerveModuleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(
        //     new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        // SwerveDriveKinematics.desaturateWheelSpeeds(
        //     swerveModuleStates, Swerve.MAX_SPEED_METERS_PER_SECOND);
        // frontLeftModule.setDesiredState(swerveModuleStates[0]);
        // frontRightModule.setDesiredState(swerveModuleStates[1]);
        // backLeftModule.setDesiredState(swerveModuleStates[2]);
        // backRightModule.setDesiredState(swerveModuleStates[3]);
    // }

    /**
     * Calculates ChassisSpeed object based on current module states.
     * 
     * @return ChassisSpeeds WPILIB Object representing current module states.
     */
    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = Swerve.DRIVE_KINEMATICS.toChassisSpeeds(
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState());
        return speeds;
    }

    /**
     * Locks the wheels in an X pattern to resist pushing and prevent the robot from
     * moving.
     */
    public void setX() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
        frontRightModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        backLeftModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        backRightModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    }

    /**
     * Points all wheels straight forward (0 degrees) with zero drive speed.
     */
    public void setToZero() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        frontRightModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        backLeftModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        backRightModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
    }

    /**
     * Points all wheels sideways (90 degrees) with zero drive speed.
     */
    public void setToNinety() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(90.0)));
        frontRightModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(90.0)));
        backLeftModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(90.0)));
        backRightModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(90.0)));
    }

    /**
     * Directly sets all four swerve module states.
     * Desaturates wheel speeds before applying to stay within the allowed maximum.
     *
     * @param desiredStates Array of four {@link SwerveModuleState} objects in
     *                      front-left, front-right, back-left, back-right order.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.MAX_SPEED_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets all four turn encoders to zero.
     * Should only be used when the wheels are physically aligned straight forward.
     */
    public void resetTurnEncoders() {
        frontLeftModule.resetTurnEncoder();
        frontRightModule.resetTurnEncoder();
        backLeftModule.resetTurnEncoder();
        backRightModule.resetTurnEncoder();
    }

    /**
     * Zeros the gyro heading so the current robot orientation becomes 0 degrees.
     */
    public void zeroHeading() {
        gyro.setYaw(0);
    }

    /**
     * Returns the current heading of the robot in degrees.
     *
     * @return Robot heading in degrees, from -180 to 180.
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()).getDegrees();
    }

    /**
     * Calculates a proportional rotation correction using the Limelight's TX
     * offset,
     * used to automatically rotate the robot toward a detected target.
     *
     * @return Rotation speed correction value, or 0.0 if no target is detected.
     */
    public double limelightAimRotate() {
        if (Robot.getLimelight().getTXIsDetected()) {
            return Robot.getLimelight().getLastValidTX() * Constants.Limelight.ROTATE_P_LIMELIGHT;
        } else {
            return 0.0;
        }
    }

    /**
     * Switches the Limelight to the specified pipeline.
     *
     * @param value The pipeline index to switch to.
     */
    public void changePipeline(int value) {
        Robot.getLimelight().setPipeline(value);
    }

    /**
     * Calculates a proportional rotation correction to drive the robot back to 0
     * degrees
     * using the gyro heading error.
     *
     * @return Rotation speed correction value toward 0 degrees.
     */
    public double gyroZeroProportional() {
        return gyroZeroError() * Constants.Limelight.ROTATE_P_GYRO;
    }

    /**
     * Calculates the heading error between the robot's current heading and 0
     * degrees.
     * The result is normalized to the range [-180, 180].
     *
     * @return Heading error in degrees.
     */
    public double gyroZeroError() {
        double headingCorrection = getHeading() % 360.0;
        if (Math.abs(headingCorrection) >= 180) {
            if (headingCorrection > 0) {
                headingCorrection -= 360;
            } else {
                headingCorrection += 360;
            }
        }
        return headingCorrection;
    }

    /**
     * Calculates a proportional lateral (strafe) correction using the Limelight's
     * horizontal TX offset for tower alignment.
     *
     * @return Strafe correction value, or 0.0 if no target is detected.
     */
    public double limelightTowerAlignTXCorrection() {
        if (Robot.getLimelight().getTXIsDetected()) {
            return limelightTowerAlignTXError() * Constants.Limelight.STRAFE_X_P;
        } else {
            return 0.0;
        }
    }

    /**
     * Calculates a proportional forward/backward correction using the Limelight's
     * vertical TY offset for tower alignment.
     *
     * @return Drive correction value, or 0.0 if no target is detected.
     */
    public double limelightTowerTYCorrection() {
        if (Robot.getLimelight().getTYIsDetected()) {
            return limelightTowerAlignTYError() * Constants.Limelight.STRAFE_Y_P;
        } else {
            return 0.0;
        }
    }

    /**
     * Returns the Limelight's last valid horizontal offset (TX), used as the
     * error input for tower alignment strafe correction.
     *
     * @return Horizontal TX offset in degrees.
     */
    public double limelightTowerAlignTXError() {
        return Robot.getLimelight().getLastValidTX();
    }

    /**
     * Returns the Limelight's last valid vertical offset (TY), used as the
     * error input for tower alignment forward/backward correction.
     *
     * @return Vertical TY offset in degrees.
     */
    public double limelightTowerAlignTYError() {
        return Robot.getLimelight().getLastValidTY();
    }

    /**
     * Drives the robot straight in the x direction (forward/backward) while
     * applying
     * gyro-based rotation correction to keep the heading locked.
     * Uses 0-degree correction or 180-degree correction depending on
     * {@link #state180}.
     *
     * @param speed Forward/backward speed from -1.0 to 1.0.
     */
    public void adjustXStraightDrive(double speed) {
        double xCorrection = speed;
        double rotCorrection;
        if (!state180) {
            rotCorrection = gyroZeroProportional();
        } else {
            rotCorrection = gyro180Proportional();
        }

        drive(xCorrection, 0.0, rotCorrection, true, false, false, false);
    }

    /**
     * Drives the robot straight in the y direction (left/right) while applying
     * gyro-based rotation correction to keep the heading locked.
     * Uses 0-degree correction or 180-degree correction depending on
     * {@link #state180}.
     *
     * @param speed Left/right speed from -1.0 to 1.0.
     */
    public void adjustYStraightDrive(double speed) {
        double yCorrection = speed;
        double rotCorrection;
        if (!state180) {
            rotCorrection = gyroZeroProportional();
        } else {
            rotCorrection = gyro180Proportional();
        }

        drive(0.0, yCorrection, rotCorrection, true, false, false, false);
    }

    /**
     * Calculates a proportional rotation correction to drive the robot back to 180
     * degrees
     * using the gyro heading error.
     *
     * @return Rotation speed correction value toward 180 degrees.
     */
    public double gyro180Proportional() {
        return gyro180Error() * Constants.Limelight.ROTATE_P_GYRO;
    }

    /**
     * Calculates the heading error between the robot's current heading and 180
     * degrees.
     * The result is normalized to the range [-180, 180].
     *
     * @return Heading error in degrees.
     */
    public double gyro180Error() {
        double headingCorrection = (getHeading() - 180.0) % 360.0;
        if (Math.abs(headingCorrection) >= 180) {
            if (headingCorrection > 0) {
                headingCorrection -= 360;
            } else {
                headingCorrection += 360;
            }
        }
        return headingCorrection;
    }

    /**
     * Returns whether the robot's target heading is currently set to 180 degrees.
     *
     * @return {@code true} if the robot is targeting 180 degrees, {@code false} for
     *         0 degrees.
     */
    public boolean get180State() {
        return state180;
    }

    /**
     * Sets whether the robot's target heading should be 180 degrees or 0 degrees.
     * Used to switch the gyro correction mode in {@link #adjustXStraightDrive} and
     * {@link #adjustYStraightDrive}.
     *
     * @param newState {@code true} to target 180 degrees, {@code false} to target 0
     *                 degrees.
     */
    public void set180State(boolean newState) {
        state180 = newState;
    }


    public static double angleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    public static double stepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = wrapAngle(_current);
        _target = wrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);
        
        if (difference <= _stepsize) { 
            return _target; 
        } else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2*Math.PI - _target < _stepsize || _target + 2*Math.PI - _current < _stepsize) { 
                return _target; 
            } else { 
                return wrapAngle(_current - stepDirection * _stepsize); /*this will handle wrapping gracefully*/ 
            }
        } else { 
            return _current + stepDirection * _stepsize; 
        }
    }

    public static double wrapAngle(double _angle) {
        double twoPi = 2*Math.PI;

        if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        } else if (_angle > twoPi) {
            return _angle - twoPi*Math.floor(_angle / twoPi);
        } else if (_angle < 0.0) {
            return _angle + twoPi*(Math.floor((-_angle) / twoPi)+1);
        } else {
            return _angle;
        }
    }

 }