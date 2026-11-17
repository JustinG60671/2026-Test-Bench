package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.opencv.core.Mat;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Swerve;

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

 }