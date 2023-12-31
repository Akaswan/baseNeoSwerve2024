package frc.robot.subsystems.templates;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.templates.IntakeSubsystem.IntakeSubsystemState;
import frc.robot.subsystems.templates.IntakeSubsystem.IntakeSubsystemType;
import frc.robot.subsystems.templates.ServoSubsystem.ServoSubsystemState;
import frc.robot.subsystems.templates.ServoSubsystem.ServoSubsystemType;

public class SubsystemConstants {

  public static class CANSparkMaxConstants {
    public int kID = 0;
    public IdleMode kIdleMode = IdleMode.kBrake;
    public MotorType kMotorType = MotorType.kBrushless;
    public int kCurrentLimit = 0;
    public boolean kInverted = false;
  }

  public static class ServoSubsystemConstants {

    // Subsystem Constants \\
    public String kName = "ERROR_ASSIGN_A_NAME";

    public ServoSubsystemType kSubsystemType = null;

    public CANSparkMaxConstants kMasterConstants = new CANSparkMaxConstants();
    public CANSparkMaxConstants[] kSlaveConstants = new CANSparkMaxConstants[0];

    public ServoSubsystemState kInitialState = null;
    public ServoSubsystemState kManualState = null;
    public ServoSubsystemState kTransitionState = null;

    // Servo Motor Subsystem Constants \\
    public double kHomePosition = 0.0;
    public double kPositionConversionFactor =
        1.0; // To find degrees: 360/gear ration ex 360/100 for 100:1

    // PID Constants
    public double kKp = 0.0;
    public double kKi = 0.0;
    public double kKd = 0.0;

    public double kSetpointTolerance = 0.0; // Tolerance for atSetpoint()
    public double kSmartMotionTolerance = 0.0; // Tolerance for pid smart motion (Stops ocilation)

    public int kDefaultSlot =
        0; // PID Slot, make more if more than one set of pid constants are used

    public double kMaxVelocity = 0.0; // Max velocity for smart motion
    public double kMaxAcceleration = 0.0; // Max acceleration for smart motion

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

  public static class IntakeSubsystemConstants {

    // Subsystem Constants \\
    public String kName = "ERROR_ASSIGN_A_NAME";

    public IntakeSubsystemType kSubsystemType = null;

    public CANSparkMaxConstants kMasterConstants = new CANSparkMaxConstants();
    public CANSparkMaxConstants[] kSlaveConstants = new CANSparkMaxConstants[0];

    public IntakeSubsystemState kInitialState = null;
  }
}
