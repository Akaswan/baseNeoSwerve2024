// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.superstructure.Arm.ArmState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.wrist.Wrist.WristState;
import frc.robot.subsystems.superstructure.wrist.WristIntake.WristIntakeState;
import frc.robot.subsystems.templates.IntakeSubsystem.IntakeSubsystemType;
import frc.robot.subsystems.templates.ServoSubsystemSparkMax.ServoSubsystemType;
import frc.robot.subsystems.templates.SubsystemConstants.IntakeSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.ServoSubsystemSparkMaxConstants;
import frc.robot.subsystems.templates.SubsystemConstants.SparkMaxConstants;
import frc.robot.utilities.SwerveModuleConstants;

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

  public static final boolean kTuningMode = true;

  // // DRIVEBASE \\ \\

  // Make sure to measure these with as much presicion as possible, as it will have great affect on
  // path planner autos and teleop driving

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
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
  }

  public static final class ArmConstants {

    public static final SparkMaxConstants kArmMasterConstants = new SparkMaxConstants();

    static {
      kArmMasterConstants.kID = 5;
      kArmMasterConstants.kIdleMode = IdleMode.kBrake;
      kArmMasterConstants.kMotorType = MotorType.kBrushless;
      kArmMasterConstants.kCurrentLimit = 80;
      kArmMasterConstants.kInverted = false;
    }

    public static final SparkMaxConstants[] kArmSlaveConstants = new SparkMaxConstants[1];

    static {
      kArmSlaveConstants[0] = new SparkMaxConstants();
      kArmSlaveConstants[0].kID = 20;
      kArmSlaveConstants[0].kIdleMode = IdleMode.kBrake;
      kArmSlaveConstants[0].kMotorType = MotorType.kBrushless;
      kArmSlaveConstants[0].kCurrentLimit = 80;
      kArmSlaveConstants[0].kInverted = false;
    }

    public static final ServoSubsystemSparkMaxConstants kArmConstants =
        new ServoSubsystemSparkMaxConstants();

    static {
      kArmConstants.kName = "Arm";

      kArmConstants.kSubsystemType = ServoSubsystemType.ARM;

      kArmConstants.kMasterConstants = kArmMasterConstants;
      kArmConstants.kSlaveConstants = kArmSlaveConstants;

      kArmConstants.kHomePosition = 0.0;
      kArmConstants.kPositionConversionFactor = 360 / 100;

      kArmConstants.kKp = 0.2;
      kArmConstants.kKi = 0.0;
      kArmConstants.kKd = 0.0;
      kArmConstants.kSetpointTolerance = 0.1;
      kArmConstants.kSmartMotionTolerance = 0.1;

      kArmConstants.kDefaultSlot = 0;

      kArmConstants.kMaxVelocity = 720;
      kArmConstants.kMaxAcceleration = 700;

      kArmConstants.kKs = 0.0;
      kArmConstants.kKg = 0.0;
      kArmConstants.kKv = 0.0;
      kArmConstants.kKa = 0.0;

      kArmConstants.kMaxPosition = 108;
      kArmConstants.kMinPosition = -8.5;

      kArmConstants.kManualAxis = XboxController.Axis.kRightY.value;
      kArmConstants.kManualMultiplier = 1;
      kArmConstants.kManualDeadBand = .1;

      kArmConstants.kInitialState = ArmState.HOME;
      kArmConstants.kManualState = ArmState.MANUAL;
      kArmConstants.kTransitionState = ArmState.TRANSITION;
    }
  }

  public static final class ElevatorConstants {

    public static final SparkMaxConstants kElevatorMasterConstants = new SparkMaxConstants();

    static {
      kElevatorMasterConstants.kID = 6;
      kElevatorMasterConstants.kIdleMode = IdleMode.kBrake;
      kElevatorMasterConstants.kMotorType = MotorType.kBrushless;
      kElevatorMasterConstants.kCurrentLimit = 80;
      kElevatorMasterConstants.kInverted = false;
    }

    public static final SparkMaxConstants[] kElevatorSlaveConstants = new SparkMaxConstants[1];

    static {
      kElevatorSlaveConstants[0] = new SparkMaxConstants();
      kElevatorSlaveConstants[0].kID = 40;
      kElevatorSlaveConstants[0].kIdleMode = IdleMode.kBrake;
      kElevatorSlaveConstants[0].kMotorType = MotorType.kBrushless;
      kElevatorSlaveConstants[0].kCurrentLimit = 80;
      kElevatorSlaveConstants[0].kInverted = false;
    }

    public static final ServoSubsystemSparkMaxConstants kElevatorConstants =
        new ServoSubsystemSparkMaxConstants();

    static {
      kElevatorConstants.kName = "Elevator";

      kElevatorConstants.kSubsystemType = ServoSubsystemType.ELEVATOR;

      kElevatorConstants.kMasterConstants = kElevatorMasterConstants;
      kElevatorConstants.kSlaveConstants = kElevatorSlaveConstants;

      kElevatorConstants.kHomePosition = 0.0;
      kElevatorConstants.kPositionConversionFactor = 10;

      kElevatorConstants.kKp = 0.2;
      kElevatorConstants.kKi = 0.0;
      kElevatorConstants.kKd = 0.0;
      kElevatorConstants.kSetpointTolerance = 0.1;
      kElevatorConstants.kSmartMotionTolerance = 0.1;

      kElevatorConstants.kDefaultSlot = 0;

      kElevatorConstants.kMaxVelocity = 10;
      kElevatorConstants.kMaxAcceleration = 7.5;

      kElevatorConstants.kKs = 0.0;
      kElevatorConstants.kKg = 0.0;
      kElevatorConstants.kKv = 0.0;
      kElevatorConstants.kKa = 0.0;

      kElevatorConstants.kMaxPosition = 1;
      kElevatorConstants.kMinPosition = 0.0;

      kElevatorConstants.kManualAxis = XboxController.Axis.kLeftY.value;
      kElevatorConstants.kManualMultiplier = .05;
      kElevatorConstants.kManualDeadBand = .1;

      kElevatorConstants.kInitialState = ElevatorState.HOME;
      kElevatorConstants.kManualState = ElevatorState.MANUAL;
      kElevatorConstants.kTransitionState = ElevatorState.TRANSITION;
    }
  }

  public static final class WristConstants {

    public static final SparkMaxConstants kWristMasterConstants = new SparkMaxConstants();

    static {
      kWristMasterConstants.kID = 7;
      kWristMasterConstants.kIdleMode = IdleMode.kBrake;
      kWristMasterConstants.kMotorType = MotorType.kBrushless;
      kWristMasterConstants.kCurrentLimit = 80;
      kWristMasterConstants.kInverted = false;
    }

    public static final SparkMaxConstants[] kWristSlaveConstants = new SparkMaxConstants[0];

    public static final ServoSubsystemSparkMaxConstants kWristConstants =
        new ServoSubsystemSparkMaxConstants();

    static {
      kWristConstants.kName = "Wrist";

      kWristConstants.kSubsystemType = ServoSubsystemType.WRIST;

      kWristConstants.kMasterConstants = kWristMasterConstants;
      kWristConstants.kSlaveConstants = kWristSlaveConstants;

      kWristConstants.kHomePosition = 155;
      kWristConstants.kPositionConversionFactor = 360 / 100;

      kWristConstants.kKp = 0.2;
      kWristConstants.kKi = 0.0;
      kWristConstants.kKd = 0.0;
      kWristConstants.kSetpointTolerance = 0.1;
      kWristConstants.kSmartMotionTolerance = 0.1;

      kWristConstants.kDefaultSlot = 0;

      kWristConstants.kMaxVelocity = 2000;
      kWristConstants.kMaxAcceleration = 2000;

      kWristConstants.kKs = 0.0;
      kWristConstants.kKg = 0.0;
      kWristConstants.kKv = 0.0;
      kWristConstants.kKa = 0.0;

      kWristConstants.kMaxPosition = 155;
      kWristConstants.kMinPosition = -85;

      kWristConstants.kManualAxis = XboxController.Axis.kRightX.value;
      kWristConstants.kManualMultiplier = 1;
      kWristConstants.kManualDeadBand = .1;

      kWristConstants.kInitialState = WristState.HOME;
      kWristConstants.kManualState = WristState.MANUAL;
      kWristConstants.kTransitionState = WristState.TRANSITION;
    }

    public static final SparkMaxConstants kWristIntakeMasterConstants = new SparkMaxConstants();

    static {
      kWristMasterConstants.kID = 45;
      kWristMasterConstants.kIdleMode = IdleMode.kBrake;
      kWristMasterConstants.kMotorType = MotorType.kBrushless;
      kWristMasterConstants.kCurrentLimit = 80;
      kWristMasterConstants.kInverted = false;
    }

    public static final SparkMaxConstants[] kWristIntakeSlaveConstants = new SparkMaxConstants[0];

    public static final IntakeSubsystemConstants kWristIntakeConstants =
        new IntakeSubsystemConstants();

    static {
      kWristIntakeConstants.kName = "WristIntake";

      kWristIntakeConstants.kSubsystemType = IntakeSubsystemType.WRIST_INTAKE;

      kWristIntakeConstants.kMasterConstants = kWristIntakeMasterConstants;
      kWristIntakeConstants.kSlaveConstants = kWristIntakeSlaveConstants;

      kWristIntakeConstants.kInitialState = WristIntakeState.IDLE;
    }
  }

  // public static final class TalonFXElevatorConstants {

  //   public static final TalonFXConstants kTalonFXElevatorMasterConstants = new
  // TalonFXConstants();

  //   static {
  //     kTalonFXElevatorMasterConstants.kID = 69;
  //     kTalonFXElevatorMasterConstants.kMaxVelocity = 50.0;
  //     kTalonFXElevatorMasterConstants.kMaxAcceleration = 45.0;
  //     kTalonFXElevatorMasterConstants.kMaxJerk = 5.0;
  //     kTalonFXElevatorMasterConstants.kKp = 0.5;
  //     kTalonFXElevatorMasterConstants.kKi = 0.2;
  //     kTalonFXElevatorMasterConstants.kKd = 0.3;
  //     kTalonFXElevatorMasterConstants.kKs = 0.2;
  //     kTalonFXElevatorMasterConstants.kKg = 0.1;
  //     kTalonFXElevatorMasterConstants.kKv = 0.1;
  //     kTalonFXElevatorMasterConstants.kKa = 0.2;
  //     kTalonFXElevatorMasterConstants.kGravityType = GravityTypeValue.Elevator_Static;
  //     kTalonFXElevatorMasterConstants.kNuetralMode = NeutralModeValue.Brake;
  //   }

  //   public static final TalonFXConstants[] kTalonFXElevatorSlaveConstants = new
  // TalonFXConstants[1];

  //   static {
  //     kTalonFXElevatorSlaveConstants[0] = new TalonFXConstants();
  //     kTalonFXElevatorSlaveConstants[0].kID = 68;
  //     kTalonFXElevatorSlaveConstants[0].kMaxVelocity = 50.0;
  //     kTalonFXElevatorSlaveConstants[0].kMaxAcceleration = 45.0;
  //     kTalonFXElevatorSlaveConstants[0].kMaxJerk = 5.0;
  //     kTalonFXElevatorSlaveConstants[0].kKp = 0.5;
  //     kTalonFXElevatorSlaveConstants[0].kKi = 0.2;
  //     kTalonFXElevatorSlaveConstants[0].kKd = 0.3;
  //     kTalonFXElevatorSlaveConstants[0].kKs = 0.2;
  //     kTalonFXElevatorSlaveConstants[0].kKg = 0.1;
  //     kTalonFXElevatorSlaveConstants[0].kKv = 0.1;
  //     kTalonFXElevatorSlaveConstants[0].kKa = 0.2;
  //     kTalonFXElevatorSlaveConstants[0].kGravityType = GravityTypeValue.Elevator_Static;
  //     kTalonFXElevatorSlaveConstants[0].kNuetralMode = NeutralModeValue.Brake;
  //   }

  //   public static final ServoSubsystemTalonFXConstants kTalonFXElevatorConstants =
  //       new ServoSubsystemTalonFXConstants();

  //   static {
  //     kTalonFXElevatorConstants.kName = "TalonFXElevator";

  //     kTalonFXElevatorConstants.kSubsystemType = ServoSubsystemType.ELEVATOR;

  //     kTalonFXElevatorConstants.kMasterConstants = kTalonFXElevatorMasterConstants;
  //     kTalonFXElevatorConstants.kSlaveConstants = kTalonFXElevatorSlaveConstants;

  //     kTalonFXElevatorConstants.kHomePosition = 0.0;
  //     kTalonFXElevatorConstants.kPositionConversionFactor = 10;

  //     kTalonFXElevatorConstants.kSetpointTolerance = 0.1;

  //     kTalonFXElevatorConstants.kDefaultSlot = 0;

  //     kTalonFXElevatorConstants.kMaxPosition = 1;
  //     kTalonFXElevatorConstants.kMinPosition = 0.0;

  //     kTalonFXElevatorConstants.kManualAxis = XboxController.Axis.kLeftY.value;
  //     kTalonFXElevatorConstants.kManualMultiplier = .05;
  //     kTalonFXElevatorConstants.kManualDeadBand = .1;

  //     kTalonFXElevatorConstants.kInitialState = TalonFXElevatorState.HOME;
  //     kTalonFXElevatorConstants.kManualState = TalonFXElevatorState.MANUAL;
  //     kTalonFXElevatorConstants.kTransitionState = TalonFXElevatorState.TRANSITION;
  //   }
  // }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
