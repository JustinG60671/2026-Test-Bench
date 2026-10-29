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

}