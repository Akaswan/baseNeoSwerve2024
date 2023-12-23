// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristState;
import frc.robot.subsystems.manager.StatedSubsystem.CANSparkMaxConstants;
import frc.robot.subsystems.manager.StatedSubsystem.SubsystemConstants;
import frc.robot.subsystems.manager.StatedSubsystem.SubsystemType;
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
  public static final Mode currentMode = Mode.SIM;

  public static final boolean INFO = true;

  public static final boolean TUNING = false;

  // // DRIVEBASE \\ \\

  // Make sure to measure these with as much presicion as possible, as it will have great affect on
  // path planner autos and teleop driving

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
  }

  public static final class DriveConstants {
    public static final double TRACK_WIDTH =
        Units.inchesToMeters(20.67); // Distance between centers of right and left wheels on robot

    public static final double WHEEL_BASE =
        Units.inchesToMeters(20.75); // Distance between front and back wheels on robot

    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(Units.inchesToMeters(32) / 2, Units.inchesToMeters(32) / 2);

    public static final double MAX_METERS_PER_SECOND =
        Units.feetToMeters(12.0); // Find this on sds website

    public static final double DRIVE_GEAR_RATIO = 8.14; // MK4i L1 Neo, find on sds website

    public static final double TURN_GEAR_RATIO =
        150.0 / 7.0; // MK4i turning ratio MK4i Neo, find on sds website

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.79); // Wheel diameter

    public static final double REGULAR_SPEED = 1; // Regular speed multiplier of robot

    public static final double SLOW_SPEED = 0.4; // Slow speed multiplier of robot

    // FRONT LEFT MODULE \\

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 12;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 52.03; // 52.03: Grimes House
    public static final SwerveModuleConstants FRONT_LEFT_MODULE =
        new SwerveModuleConstants(
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET);

    // FRONT RIGHT MODULE \\

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 18;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 17;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 7;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 144.22; // 144.22: Grimes House
    public static final SwerveModuleConstants FRONT_RIGHT_MODULE =
        new SwerveModuleConstants(
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET);

    // BACK LEFT MODULE \\

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 45.25; // 45.25: Grimes House
    public static final SwerveModuleConstants BACK_LEFT_MODULE =
        new SwerveModuleConstants(
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET);

    // BACK RIGHT MODULE \\

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 5;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 231.1; // 231.1: Grimes House
    public static final SwerveModuleConstants BACK_RIGHT_MODULE =
        new SwerveModuleConstants(
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET);

    public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(5.0, 0, 0);
    public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(5.0, 0, 0);
  }

  public static final class ArmConstants {

    public static final CANSparkMaxConstants kArmMasterConstants = new CANSparkMaxConstants();

    static {
      kArmMasterConstants.kID = 5;
      kArmMasterConstants.kIdleMode = IdleMode.kBrake;
      kArmMasterConstants.kMotorType = MotorType.kBrushless;
      kArmMasterConstants.kCurrentLimit = 80;
    }

    public static final CANSparkMaxConstants[] kArmSlaveConstants = new CANSparkMaxConstants[1];

    static {
      kArmSlaveConstants[0] = new CANSparkMaxConstants();
      kArmSlaveConstants[0].kID = 20;
      kArmSlaveConstants[0].kIdleMode = IdleMode.kBrake;
      kArmSlaveConstants[0].kMotorType = MotorType.kBrushless;
      kArmSlaveConstants[0].kCurrentLimit = 80;
    }

    public static final SubsystemConstants kArmConstants = new SubsystemConstants();

    static {
      kArmConstants.kName = "Arm";

      kArmConstants.kSubsystemType = SubsystemType.ARM;

      kArmConstants.kMasterConstants = kArmMasterConstants;
      kArmConstants.kSlaveConstants = kArmSlaveConstants;

      kArmConstants.kHomePosition = 0.0;
      kArmConstants.kRotationsPerUnitDistance = 360 / 100;

      kArmConstants.kKp = 0.2;
      kArmConstants.kKi = 0.0;
      kArmConstants.kKd = 0.0;
      kArmConstants.kSetpointTolerance = 0.1;
      kArmConstants.kSmartMotionTolerance = 0.1;

      kArmConstants.kDefaultSlot = 0;

      kArmConstants.kMaxVelocity = 200;
      kArmConstants.kMaxAcceleration = 200;

      kArmConstants.kKs = 0.0;
      kArmConstants.kKg = 0.0;
      kArmConstants.kKv = 0.0;
      kArmConstants.kKa = 0.0;

      kArmConstants.kMaxPosition = 150.0;
      kArmConstants.kMinPosition = 0.0;

      kArmConstants.kManualAxis = XboxController.Axis.kRightY.value;
      kArmConstants.kManualMultiplier = 1;
      kArmConstants.kManualDeadZone = .1;

      kArmConstants.kInitialState = ArmState.HOME;
      kArmConstants.kManualState = ArmState.MANUAL;
      kArmConstants.kTransitionState = ArmState.TRANSITION;
      kArmConstants.kSetpointSwitchState = ArmState.SETPOINT_SWITCH;
    }
  }

  public static final class ElevatorConstants {

    public static final CANSparkMaxConstants kElevatorMasterConstants = new CANSparkMaxConstants();

    static {
      kElevatorMasterConstants.kID = 6;
      kElevatorMasterConstants.kIdleMode = IdleMode.kBrake;
      kElevatorMasterConstants.kMotorType = MotorType.kBrushless;
      kElevatorMasterConstants.kCurrentLimit = 80;
    }

    public static final CANSparkMaxConstants[] kElevatorSlaveConstants =
        new CANSparkMaxConstants[1];

    static {
      kElevatorSlaveConstants[0] = new CANSparkMaxConstants();
      kElevatorSlaveConstants[0].kID = 40;
      kElevatorSlaveConstants[0].kIdleMode = IdleMode.kBrake;
      kElevatorSlaveConstants[0].kMotorType = MotorType.kBrushless;
      kElevatorSlaveConstants[0].kCurrentLimit = 80;
    }

    public static final SubsystemConstants kElevatorConstants = new SubsystemConstants();

    static {
      kElevatorConstants.kName = "Elevator";

      kElevatorConstants.kSubsystemType = SubsystemType.ELEVATOR;

      kElevatorConstants.kMasterConstants = kElevatorMasterConstants;
      kElevatorConstants.kSlaveConstants = kElevatorSlaveConstants;

      kElevatorConstants.kHomePosition = 0.0;
      kElevatorConstants.kRotationsPerUnitDistance = 10;

      kElevatorConstants.kKp = 0.2;
      kElevatorConstants.kKi = 0.0;
      kElevatorConstants.kKd = 0.0;
      kElevatorConstants.kSetpointTolerance = 0.1;
      kElevatorConstants.kSmartMotionTolerance = 0.1;

      kElevatorConstants.kDefaultSlot = 0;

      kElevatorConstants.kMaxVelocity = 1;
      kElevatorConstants.kMaxAcceleration = .5;

      kElevatorConstants.kKs = 0.0;
      kElevatorConstants.kKg = 0.0;
      kElevatorConstants.kKv = 0.0;
      kElevatorConstants.kKa = 0.0;

      kElevatorConstants.kMaxPosition = 3.0;
      kElevatorConstants.kMinPosition = 0.0;

      kElevatorConstants.kManualAxis = XboxController.Axis.kLeftY.value;
      kElevatorConstants.kManualMultiplier = .05;
      kElevatorConstants.kManualDeadZone = .1;

      kElevatorConstants.kInitialState = ElevatorState.HOME;
      kElevatorConstants.kManualState = ElevatorState.MANUAL;
      kElevatorConstants.kTransitionState = ElevatorState.TRANSITION;
      kElevatorConstants.kSetpointSwitchState = ElevatorState.SETPOINT_SWITCH;
    }
  }

  public static final class WristConstants {

    public static final CANSparkMaxConstants kWristMasterConstants = new CANSparkMaxConstants();

    static {
      kWristMasterConstants.kID = 7;
      kWristMasterConstants.kIdleMode = IdleMode.kBrake;
      kWristMasterConstants.kMotorType = MotorType.kBrushless;
      kWristMasterConstants.kCurrentLimit = 80;
    }

    public static final CANSparkMaxConstants[] kWristSlaveConstants = new CANSparkMaxConstants[0];

    public static final SubsystemConstants kWristConstants = new SubsystemConstants();

    static {
      kWristConstants.kName = "Wrist";

      kWristConstants.kSubsystemType = SubsystemType.WRIST;

      kWristConstants.kMasterConstants = kWristMasterConstants;
      kWristConstants.kSlaveConstants = kWristSlaveConstants;

      kWristConstants.kHomePosition = 0.0;
      kWristConstants.kRotationsPerUnitDistance = 360 / 100;

      kWristConstants.kKp = 0.2;
      kWristConstants.kKi = 0.0;
      kWristConstants.kKd = 0.0;
      kWristConstants.kSetpointTolerance = 0.1;
      kWristConstants.kSmartMotionTolerance = 0.1;

      kWristConstants.kDefaultSlot = 0;

      kWristConstants.kMaxVelocity = 150;
      kWristConstants.kMaxAcceleration = 140;

      kWristConstants.kKs = 0.0;
      kWristConstants.kKg = 0.0;
      kWristConstants.kKv = 0.0;
      kWristConstants.kKa = 0.0;

      kWristConstants.kMaxPosition = 160;
      kWristConstants.kMinPosition = -160;

      kWristConstants.kManualAxis = XboxController.Axis.kRightX.value;
      kWristConstants.kManualMultiplier = 1;
      kWristConstants.kManualDeadZone = .1;

      kWristConstants.kInitialState = WristState.HOME;
      kWristConstants.kManualState = WristState.MANUAL;
      kWristConstants.kTransitionState = WristState.TRANSITION;
      kWristConstants.kSetpointSwitchState = WristState.SETPOINT_SWITCH;
    }
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
