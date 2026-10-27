package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import static frc.robot.Constants.Swerve.*;

/**
 * Represents a single swerve drive module, handling both drive and steering motor control.
 * Each module has a TalonFX drive motor, a TalonFX turn motor, and a CANcoder for absolute
 * steering angle feedback. Uses Motion Magic for both velocity (drive) and position (turn) control.
 */
public class SwerveModule extends SubsystemBase {

    /** Motor that drives the wheel forward and backward. */
    private TalonFX driveMotor;

    /** Motor that steers the wheel left and right. */
    private TalonFX turnMotor;

    /** Current drive motor position in rotations, updated every loop. */
    public double driveValue;

    /** Absolute encoder used to measure the steering angle of this module. */
    private CANcoder turnEncoder;

    /** Current steering angle in degrees, updated every loop. */
    public double turnEncoderValue = 0;

    /** Current steering angle in rotations, updated every loop. */
    public double turnEncoderRotation = 0;

    /** Full configuration object for the drive motor. */
    private TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();

    /** Full configuration object for the turn motor. */
    private TalonFXConfiguration turnTalonConfig = new TalonFXConfiguration();

    /** Motion Magic position control request used to steer the module to a target angle. */
    MotionMagicVoltage turnMMVoltage = new MotionMagicVoltage(0).withSlot(1);

    /** Motion Magic velocity control request used to drive the module at a target speed. */
    MotionMagicVelocityVoltage driveMMVoltage = new MotionMagicVelocityVoltage(0).withSlot(0);

    /** Motion Magic settings pulled from the drive motor config object. */
    MotionMagicConfigs driveMMConfig = driveTalonConfig.MotionMagic;

    /** Motion Magic settings pulled from the turn motor config object. */
    MotionMagicConfigs turnMMConfig = turnTalonConfig.MotionMagic;

    /** Slot 0 PID and feedforward gains configuration for the drive motor. */
    private Slot0Configs driveSlotConfig = driveTalonConfig.Slot0;

    /** Slot 1 PID and feedforward gains configuration for the turn motor. */
    private Slot1Configs turnSlotConfig = turnTalonConfig.Slot1;

    /** The most recently commanded state for this module (speed and angle). */
    public SwerveModuleState desiredState;

    /**
     * Constructs a SwerveModule. Sets up the drive motor, turn motor, and CANcoder,
     * then applies all PID gains, Motion Magic parameters, current limits, and
     * encoder configuration. Resets the drive encoder at the end of initialization.
     *
     * @param driveMotorID  CAN ID for the drive motor.
     * @param turnMotorID   CAN ID for the turn motor.
     * @param turnEncoderID CAN ID for the CANcoder used for steering feedback.
     * @param canbusName    Name of the CAN bus all three devices are on.
     * @param magnetOffset  Absolute encoder offset in rotations, used to zero the steering angle.
     * @param driveInverted Whether the drive motor should spin counter-clockwise for forward motion.
     */
    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderID, String canbusName, double magnetOffset, boolean driveInverted) {

        // Default to stopped with wheels facing forward
        desiredState = new SwerveModuleState(0.0, new Rotation2d());

        // Set up the CAN bus and all three devices
        CANBus canbus = new CANBus(canbusName);
        driveMotor = new TalonFX(driveMotorID, canbus);
        turnMotor = new TalonFX(turnMotorID, canbus);
        driveValue = 0;
        turnEncoder = new CANcoder(turnEncoderID, canbus);

        // Apply the magnet offset and set clockwise as the positive direction for the encoder
        CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
        encoderConfigs.MagnetSensor.MagnetOffset = magnetOffset;
        encoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        turnEncoder.getConfigurator().apply(encoderConfigs);

        // Configure the turn motor to use the CANcoder as remote feedback with continuous wrap
        // so the controller always takes the shortest path to the target angle
        turnTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnTalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnTalonConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnTalonConfig.CurrentLimits.SupplyCurrentLimit = TURNING_MOTOR_CURRENT_LIMIT;
        turnTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Configure turn motor PID and feedforward gains
        turnSlotConfig.kP = TURN_P;
        turnSlotConfig.kI = TURN_I;
        turnSlotConfig.kD = TURN_D;
        turnSlotConfig.kS = TURN_S;
        turnSlotConfig.kV = TURN_V;
        turnSlotConfig.kA = TURN_A;

        // Configure turn motor Motion Magic cruise velocity, acceleration, and jerk limits
        turnMMConfig.MotionMagicCruiseVelocity = TURN_MM_CRUISE_VELOCITY;
        turnMMConfig.MotionMagicAcceleration = TURN_MM_ACCELERATION;
        turnMMConfig.MotionMagicJerk = TURN_MM_JERK;

        // Push the turn configuration to the turn motor
        turnMotor.getConfigurator().apply(turnTalonConfig);

        // Configure drive motor PID and feedforward gains
        driveSlotConfig.kP = DRIVE_P;
        driveSlotConfig.kI = DRIVE_I;
        driveSlotConfig.kD = DRIVE_D;
        driveSlotConfig.kS = DRIVE_S;
        driveSlotConfig.kV = DRIVE_V;

        // Apply drive motor config — inversion direction varies per module based on physical orientation
        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveTalonConfig.MotorOutput.Inverted = driveInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        driveTalonConfig.CurrentLimits.SupplyCurrentLimit = DRIVING_MOTOR_CURRENT_LIMIT;
        driveMMConfig.MotionMagicAcceleration = DRIVE_MM_ACCELERATION;
        driveMMConfig.MotionMagicJerk = DRIVE_MM_JERK;

        // Push the drive configuration to the drive motor
        driveMotor.getConfigurator().apply(driveTalonConfig);

        // Push the PID slot configs separately after the base configs
        turnMotor.getConfigurator().apply(turnSlotConfig);
        driveMotor.getConfigurator().apply(driveSlotConfig);

        // Zero the drive encoder on startup
        resetDriveEncoder();
    }

    /**
     * Runs every robot loop iteration. Updates the drive position,
     * steering angle in degrees, and steering angle in rotations.
     */
    @Override
    public void periodic() {
        driveValue = driveMotor.getPosition().getValueAsDouble() * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS / Constants.Swerve.DRIVING_MOTOR_REDUCTION;
        turnEncoderValue = getAngle();
        turnEncoderRotation = turnEncoder.getPosition().getValueAsDouble();
    }

    /**
     * Returns the current state of this module as a {@link SwerveModuleState},
     * containing the current drive velocity and steering angle.
     *
     * @return The current {@link SwerveModuleState} of this module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(turnEncoderValue));
    }

    /**
     * Returns the current position of this module as a {@link SwerveModulePosition},
     * containing the total drive distance traveled and current steering angle.
     *
     * @return The current {@link SwerveModulePosition} of this module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveValue, new Rotation2d(turnEncoderValue));
    }

    /**
     * Commands the module to a desired speed and angle.
     * Optimizes the steering angle so the module never rotates more than 90 degrees,
     * reversing drive direction instead if needed.
     *
     * @param inputDesiredState The desired {@link SwerveModuleState} with target speed and angle.
     */
    public void setDesiredState(SwerveModuleState inputDesiredState) {
        // Optimize the target angle to avoid spinning more than 90 degrees
        inputDesiredState.optimize(Rotation2d.fromDegrees(turnEncoderValue));

        desiredState = inputDesiredState;

        // Convert angle from degrees to rotations for the turn motor controller
        turnMotor.setControl(turnMMVoltage.withPosition(desiredState.angle.getDegrees() / 360.0));

        // Convert speed from m/s to motor rotations per second using gear ratio and wheel circumference
        driveMotor.setControl(driveMMVoltage.withVelocity(
                desiredState.speedMetersPerSecond * Constants.Swerve.DRIVING_MOTOR_REDUCTION / WHEEL_CIRCUMFERENCE_METERS));
    }

    /**
     * Resets the drive motor encoder position to zero.
     * Should be called on startup or when reseating the robot's odometry.
     */
    public void resetDriveEncoder() {
        driveMotor.setPosition(0);
    }

    /**
     * Resets the turn CANcoder position to zero.
     * Only use this if you need to manually re-zero the steering encoder.
     */
    public void resetTurnEncoder() {
        turnEncoder.setPosition(0);
    }

    /**
     * Returns the current drive velocity of this module in meters per second.
     *
     * @return Drive velocity in meters per second.
     */
    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * WHEEL_CIRCUMFERENCE_METERS / Constants.Swerve.DRIVING_MOTOR_REDUCTION;
    }

    /**
     * Returns the current steering angle of this module in degrees.
     *
     * @return Steering angle in degrees, derived from the CANcoder position.
     */
    public double getAngle() {
        return turnEncoder.getPosition().getValueAsDouble() * 360.0;
    }
}