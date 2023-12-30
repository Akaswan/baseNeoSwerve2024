package frc.robot.subsystems.manager;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StatedSubsystem extends SubsystemBase {

  protected SubsystemConstants m_constants;

  protected final CANSparkMax m_master;
  protected final CANSparkMax[] m_slaves;
  protected final RelativeEncoder m_encoder;

  protected SubsystemState m_lastHeldState = null;
  protected SubsystemState m_currentState = null;
  protected SubsystemState m_previousDesiredState = null;
  protected SubsystemState m_desiredState = null;

  public StatedSubsystem(SubsystemConstants constants) {
    m_constants = constants;

    m_master =
        new CANSparkMax(m_constants.kMasterConstants.kID, m_constants.kMasterConstants.kMotorType);
    m_master.setIdleMode(m_constants.kMasterConstants.kIdleMode);
    m_master.setSmartCurrentLimit(m_constants.kMasterConstants.kCurrentLimit);

    m_encoder = m_master.getEncoder();

    m_slaves = new CANSparkMax[m_constants.kSlaveConstants.length];

    for (int i = 0; i < m_constants.kSlaveConstants.length; i++) {
      m_slaves[i] =
          new CANSparkMax(
              m_constants.kSlaveConstants[i].kID, m_constants.kSlaveConstants[i].kMotorType);
      m_slaves[i].setIdleMode(m_constants.kSlaveConstants[i].kIdleMode);
      m_slaves[i].setSmartCurrentLimit(m_constants.kSlaveConstants[i].kCurrentLimit);
      m_slaves[i].follow(m_master);
    }

    m_lastHeldState = m_constants.kInitialState;
    m_currentState = m_constants.kInitialState;
    m_desiredState = m_constants.kInitialState;
    m_previousDesiredState = m_constants.kInitialState;

    setName(m_constants.kName);
  }

  public abstract void abstractSubsystemPeriodic();

  public abstract void subsystemPeriodic();

  public abstract void outputAbstractSubsystemTelemetry();

  public abstract void outputSusbsystemTelemetry();

  public SubsystemType getSubsystemType() {
    return m_constants.kSubsystemType;
  }

  public SubsystemConstants getConstants() {
    return m_constants;
  }

  @Override
  public void periodic() {
    abstractSubsystemPeriodic(); // Always put this one first
    subsystemPeriodic();
    outputAbstractSubsystemTelemetry();
    outputSusbsystemTelemetry();
  }

  public static class SubsystemConstants {

    // Subsystem Constants \\
    public String kName = "ERROR_ASSIGN_A_NAME";

    public SubsystemType kSubsystemType = null;

    public CANSparkMaxConstants kMasterConstants = new CANSparkMaxConstants();
    public CANSparkMaxConstants[] kSlaveConstants = new CANSparkMaxConstants[0];

    public SubsystemState kInitialState = null;
    public SubsystemState kManualState = null;
    public SubsystemState kTransitionState = null;
    public SubsystemState kSetpointSwitchState = null;

    // Servo Motor Subsystem Constants \\
    public double kHomePosition = 0.0;
    public double kRotationsPerUnitDistance =
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
    public double kManualDeadZone = 0;
  }

  public static class CANSparkMaxConstants {
    public int kID = 0;
    public IdleMode kIdleMode = IdleMode.kBrake;
    public MotorType kMotorType = MotorType.kBrushless;
    public int kCurrentLimit = 0;
  }

  public interface SubsystemState {
    String getName();

    double getPosition();

    double getVelocity();

    void setPosition(double position);

    void setVelocity(double velocity);
  }

  public enum SubsystemType {
    ARM,
    ELEVATOR,
    WRIST
  }
}
