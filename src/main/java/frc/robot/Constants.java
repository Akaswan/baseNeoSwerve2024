// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.TreeMap;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;
import frc.robot.subsystems.templates.VelocitySubsystem.VelocitySubsystemType;
import frc.robot.subsystems.templates.PositionSubsystem.PositionSubsystemType;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.SparkMaxConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;
import frc.lib.utilities.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public static final boolean kInfoMode = true;

  public static final boolean kTuningMode = false;

  // // DRIVEBASE \\ \\

  // Make sure to measure these with as much presicion as possible, as it will have great affect on
  // path planner autos and teleop driving

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
  }

  public static final class AutoConstants {
    public static Translation2d[] kNotePlacements = new Translation2d[] {
      new Translation2d(7.68, 7.44),
      new Translation2d(7.68, 5.78),
      new Translation2d(7.68, 4.10),
      new Translation2d(7.68, 2.44),
      new Translation2d(7.68, 0.78)
    };
  }

  public static final class DriveConstants {

    public static double drivekp = 0.15751;
    public static double driveki = 0.0;
    public static double drivekd = 0.0;
    public static double drivekff = 0.23983;
    public static double driverampRate = 1.0;

    public static double turnkp = 0.1;
    public static double turnki = 0.0;
    public static double turnkd = 0.0;
    public static double turnkff = 0.0;

    public static double kDriveModifier = 2;
    public static double kTurnModifier = 2;

    /* The lower this is the more you want odometry to trust the april tags
    Scales based on the percentage of an april tag in view
    Dont do anything below 0*/
    public static double kAprilTagTrustMultiplier = 1.0;

    public static int kPigeon = 0;

    public static final double kTrackWidth =
        Units.inchesToMeters(20.67); // Distance between centers of right and left wheels on robot

    public static final double kWheelBase =
        Units.inchesToMeters(20.76); // Distance between centers of front and back wheels on robot

    public static final double kDriveBaseRadius =
        Math.hypot(
            Units.inchesToMeters(32) / 2,
            Units.inchesToMeters(32)
                / 2); // Distance from center of the robot to corner of the bumpers

    public static final double kMaxMetersPerSecond =
        Units.feetToMeters(12); // Run drivebase at max speed on the ground to find top speed

    public static final double kDriveGearRatio = 8.14; // MK4i L1 Neo, find on sds website

    public static final double kTurnGearRatio =
        150.0 / 7.0; // MK4i turning ratio MK4i Neo, find on sds website

    public static final double kWheelDiameter = Units.inchesToMeters(3.79); // Wheel diameter

    public static final double kMaxRotationRadiansPerSecond =
        Math.PI * 2.0; // Just kind of find what works, this is from 930 2023

    public static final double kRegularSpeed = 1; // Regular speed multiplier of robot

    public static final double kSlowSpeed = 0.4; // Slow speed multiplier of robot

    public static final int kFrontLeftDriveMotor = 12;
    public static final int kFrontLeftSteerMotor = 11;
    public static final int kFrontLeftSteerEncoder = 1;
    public static final double kFrontLeftOffset = 52.03;
    public static final SwerveModuleConstants kFrontLeft =
        new SwerveModuleConstants(
            kFrontLeftDriveMotor, kFrontLeftSteerMotor, kFrontLeftSteerEncoder, kFrontLeftOffset);

    public static final int kFrontRightDriveMotor = 18;
    public static final int kFrontRightSteerMotor = 17;
    public static final int kFrontRightSteerEncoder = 7;
    public static final double kFrontRightSteerOffset = 144.22;
    public static final SwerveModuleConstants kFrontRight =
        new SwerveModuleConstants(
            kFrontRightDriveMotor,
            kFrontRightSteerMotor,
            kFrontRightSteerEncoder,
            kFrontRightSteerOffset);

    public static final int kBackLeftDriveMotor = 14;
    public static final int kBackLeftSteerMotor = 13;
    public static final int kBackLeftSteerEncoder = 3;
    public static final double kBackLeftSteerOffset = 45.25;
    public static final SwerveModuleConstants kBackLeft =
        new SwerveModuleConstants(
            kBackLeftDriveMotor, kBackLeftSteerMotor, kBackLeftSteerEncoder, kBackLeftSteerOffset);

    public static final int kBackRightDriveMotor = 16;
    public static final int kBackRightSteerMotor = 15;
    public static final int kBackRightSteerEncoder = 5;
    public static final double kBackRightSteerOffset = 231.1;
    public static final SwerveModuleConstants kBackRight =
        new SwerveModuleConstants(
            kBackRightDriveMotor,
            kBackRightSteerMotor,
            kBackRightSteerEncoder,
            kBackRightSteerOffset);

    public static final double kDriveRevToMeters = ((kWheelDiameter * Math.PI) / kDriveGearRatio);
    public static final double kDriveRpmToMetersPerSecond = kDriveRevToMeters / 60.0;
    public static final double kTurnRotationsToDegrees = 360.0 / kTurnGearRatio;

    public static final Translation2d[] kModuleTranslations = {
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final PIDConstants kPathPlannerTranslationPID = new PIDConstants(5.0, 0, 0);
    public static final PIDConstants kPathPlannerRotationPID = new PIDConstants(5.0, 0, 0);

    public static final SwerveModuleState[] kXWheels = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
    };

    public static final Translation2d kBlueSpeakerPosition = new Translation2d(0, 5.56);
    public static final Translation2d kRedSpeakerPosition = new Translation2d(0, 2.65);
  }

  public static final class LauncherConstants {

    // IN METERS
    public static final TreeMap<Double, Double> kDistanceRPMMap = new TreeMap<>();
    static {
      kDistanceRPMMap.put(0.0, 1000.0);
      kDistanceRPMMap.put(1.0, 2000.0);
      kDistanceRPMMap.put(1.5, 2500.0);
      kDistanceRPMMap.put(2.0, 3000.0);
      kDistanceRPMMap.put(2.5, 3500.0);
      kDistanceRPMMap.put(3.0, 4000.0);
      kDistanceRPMMap.put(3.5, 4500.0);
      kDistanceRPMMap.put(4.0, 5000.0);
      kDistanceRPMMap.put(4.5, 5500.0);
      kDistanceRPMMap.put(5.0, 6000.0);
      kDistanceRPMMap.put(5.5, 6500.0);
    }

    // IN DEGREES
    public static final TreeMap<Double, Double> kDistancePitchMap = new TreeMap<>();
    static {
      kDistancePitchMap.put(0.0, 90.0);
      kDistancePitchMap.put(1.0, 60.0);
      kDistancePitchMap.put(1.5, 55.0);
      kDistancePitchMap.put(2.0, 50.0);
      kDistancePitchMap.put(2.5, 45.0);
      kDistancePitchMap.put(3.0, 40.0);
      kDistancePitchMap.put(3.5, 35.0);
      kDistancePitchMap.put(4.0, 30.0);
      kDistancePitchMap.put(4.5, 25.0);
      kDistancePitchMap.put(5.0, 20.0);
      kDistancePitchMap.put(5.5, 15.0);
      kDistancePitchMap.put(6.0, 10.0);
    }

    public static final SparkMaxConstants kLauncherFlywheelMasterConstants = new SparkMaxConstants();

    static {
      kLauncherFlywheelMasterConstants.kID = 34;
      kLauncherFlywheelMasterConstants.kIdleMode = IdleMode.kBrake;
      kLauncherFlywheelMasterConstants.kMotorType = MotorType.kBrushless;
      kLauncherFlywheelMasterConstants.kCurrentLimit = 80;
      kLauncherFlywheelMasterConstants.kInverted = false;
    }

    public static final SparkMaxConstants[] kLauncherFlywheelSlaveConstants = new SparkMaxConstants[0];

    public static final VelocitySubsystemConstants kLauncherFlywheelConstants =
        new VelocitySubsystemConstants();

    static {
      kLauncherFlywheelConstants.kName = "Launcher Flywheel";

      kLauncherFlywheelConstants.kSubsystemType = VelocitySubsystemType.LAUNCHER_FLYWHEEL;

      kLauncherFlywheelConstants.kMasterConstants = kLauncherFlywheelMasterConstants;
      kLauncherFlywheelConstants.kSlaveConstants = kLauncherFlywheelSlaveConstants;

      kLauncherFlywheelConstants.kVelocityConversionFactor = 3 / 60; // division by 60 to get rotations per second

      kLauncherFlywheelConstants.kKp = 0.01;
      kLauncherFlywheelConstants.kKi = 0.0;
      kLauncherFlywheelConstants.kKd = 0.0;

      kLauncherFlywheelConstants.kDefaultSlot = 0;

      kLauncherFlywheelConstants.kKs = 0.0;
      kLauncherFlywheelConstants.kKv = 0.0;
      kLauncherFlywheelConstants.kKa = 0.0;

      kLauncherFlywheelConstants.kInitialState = LauncherFlywheelState.OFF;
      kLauncherFlywheelConstants.kTransitionState = LauncherFlywheelState.TRANSITION;
    }

  public static final SparkMaxConstants kLauncherWristMasterConstants = new SparkMaxConstants();

    static {
      kLauncherWristMasterConstants.kID = 7;
      kLauncherWristMasterConstants.kIdleMode = IdleMode.kBrake;
      kLauncherWristMasterConstants.kMotorType = MotorType.kBrushless;
      kLauncherWristMasterConstants.kCurrentLimit = 80;
      kLauncherWristMasterConstants.kInverted = false;
    }

    public static final SparkMaxConstants[] kWristSlaveConstants = new SparkMaxConstants[0];

    public static final PositionSubsystemConstants kLauncherWristConstants =
        new PositionSubsystemConstants();

    static {
      kLauncherWristConstants.kName = "Launcher Wrist";

      kLauncherWristConstants.kSubsystemType = PositionSubsystemType.LAUNCHER_WRIST;

      kLauncherWristConstants.kMasterConstants = kLauncherWristMasterConstants;
      kLauncherWristConstants.kSlaveConstants = kWristSlaveConstants;

      kLauncherWristConstants.kHomePosition = 155;
      kLauncherWristConstants.kPositionConversionFactor = 360 / 100;

      kLauncherWristConstants.kKp = 0.2;
      kLauncherWristConstants.kKi = 0.0;
      kLauncherWristConstants.kKd = 0.0;
      kLauncherWristConstants.kSetpointTolerance = 0.1;
      kLauncherWristConstants.kSmartMotionTolerance = 0.1;

      kLauncherWristConstants.kDefaultSlot = 0;

      kLauncherWristConstants.kMaxVelocity = 2000;
      kLauncherWristConstants.kMaxAcceleration = 2000;

      kLauncherWristConstants.kKs = 0.0;
      kLauncherWristConstants.kKg = 0.0;
      kLauncherWristConstants.kKv = 0.0;
      kLauncherWristConstants.kKa = 0.0;

      kLauncherWristConstants.kMaxPosition = 155;
      kLauncherWristConstants.kMinPosition = -85;

      kLauncherWristConstants.kManualControlMode = ManualControlMode.TRIGGERS;
      kLauncherWristConstants.kManualMultiplier = 1;
      kLauncherWristConstants.kManualDeadBand = .1;

      kLauncherWristConstants.kInitialState = LauncherWristState.DOWN;
      kLauncherWristConstants.kManualState = LauncherWristState.MANUAL;
      kLauncherWristConstants.kTransitionState = LauncherWristState.TRANSITION;
    }
  }


  public static enum ManualControlMode {
    TRIGGERS,
    BUMPERS,
    LEFT_X,
    LEFT_Y,
    RIGHT_X,
    RIGHT_Y
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
