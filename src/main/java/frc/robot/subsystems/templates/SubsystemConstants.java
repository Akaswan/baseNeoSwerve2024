package frc.robot.subsystems.templates;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.templates.VoltageSubsystem.VoltageSubsystemState;
import frc.robot.subsystems.templates.VoltageSubsystem.VoltageSubsystemType;
import frc.robot.subsystems.templates.PositionSubsystem.PositionSubsystemState;
import frc.robot.subsystems.templates.PositionSubsystem.PositionSubsystemType;
import frc.robot.subsystems.templates.VelocitySubsystem.VelocitySubsystemState;
import frc.robot.subsystems.templates.VelocitySubsystem.VelocitySubsystemType;

public class SubsystemConstants {

  public static class SparkMaxConstants {
    public int kID = 0;
    public IdleMode kIdleMode = IdleMode.kBrake;
    public MotorType kMotorType = MotorType.kBrushless;
    public int kCurrentLimit = 0;
    public boolean kInverted = false;
  }

  public static class PositionSubsystemConstants {

    // Subsystem Constants \\
    public String kName = "ERROR_ASSIGN_A_NAME";

    public PositionSubsystemType kSubsystemType = null;

    public SparkMaxConstants kMasterConstants = new SparkMaxConstants();
    public SparkMaxConstants[] kSlaveConstants = new SparkMaxConstants[0];

    public PositionSubsystemState kInitialState = null;
    public PositionSubsystemState kManualState = null;
    public PositionSubsystemState kTransitionState = null;

    // Servo Motor Subsystem Constants \\
    public double kHomePosition = 0.0;
    public double kPositionConversionFactor =
        1.0; // To find degrees: 360/gear ration ex 360/100 for 100:1

    // PID Constants
    public double kKp = 0.0;
    public double kKi = 0.0;
    public double kKd = 0.0;

    public double kSetpointTolerance = 0.0; // Tolerance for atSetpoint()
    public double kSmartMotionTolerance = 0.0;

    public int kDefaultSlot =
        0; // PID Slot, make more if more than one set of pid constants are used

    public double kMaxVelocity = 0.0; // Max velocity for motion profile
    public double kMaxAcceleration = 0.0; // Max acceleration for motion profile

    // Feedforward constants
    public double kKs = 0.0;
    public double kKg = 0.0;
    public double kKv = 0.0;
    public double kKa = 0.0;

    // Max/Min positions the subsystem should be able to move
    public double kMaxPosition = Double.POSITIVE_INFINITY;
    public double kMinPosition = Double.NEGATIVE_INFINITY;

    // Manual constants
    public int kManualAxis = 0;
    public double kManualMultiplier = 0;
    public double kManualDeadBand = 0;
  }

  public static class VoltageSubsystemConstants {

    // Subsystem Constants \\
    public String kName = "ERROR_ASSIGN_A_NAME";

    public VoltageSubsystemType kSubsystemType = null;

    public SparkMaxConstants kMasterConstants = new SparkMaxConstants();
    public SparkMaxConstants[] kSlaveConstants = new SparkMaxConstants[0];

    public VoltageSubsystemState kInitialState = null;
  }

  public static class VelocitySubsystemConstants {

    // Subsystem Constants \\
    public String kName = "ERROR_ASSIGN_A_NAME";

    public VelocitySubsystemType kSubsystemType = null;

    public SparkMaxConstants kMasterConstants = new SparkMaxConstants();
    public SparkMaxConstants[] kSlaveConstants = new SparkMaxConstants[0];

    public VelocitySubsystemState kInitialState = null;
    public VelocitySubsystemState kTransitionState = null;

    public double kVelocityConversionFactor =
        1.0; // To find degrees: 360/gear ratio ex 360/100 for 100:1

    // PID Constants
    public double kKp = 0.0;
    public double kKi = 0.0;
    public double kKd = 0.0;

    public double kSetpointTolerance = 0.0; // Tolerance for atSetpoint()

    public int kDefaultSlot =
        0; // PID Slot, make more if more than one set of pid constants are used

    // Feedforward constants
    public double kKs = 0.0;
    public double kKv = 0.0;
    public double kKa = 0.0;
  }
}
