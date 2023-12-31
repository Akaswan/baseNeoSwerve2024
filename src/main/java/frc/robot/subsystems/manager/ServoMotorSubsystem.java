// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manager;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public abstract class ServoMotorSubsystem extends StatedSubsystem {

  protected final SparkMaxPIDController m_pidController;

  protected final CANSparkMax m_master;
  protected final CANSparkMax[] m_slaves;
  protected final RelativeEncoder m_encoder;

  protected TrapezoidProfile m_profile;
  protected TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  protected TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  protected double m_profileStartTime = -1;

  protected double m_arbFeedforward = 0;

  protected ServoMotorSubsystem(final SubsystemConstants constants) {
    super(constants);

    m_master =
        new CANSparkMax(m_constants.kMasterConstants.kID, m_constants.kMasterConstants.kMotorType);
    m_master.setIdleMode(m_constants.kMasterConstants.kIdleMode);
    m_master.setSmartCurrentLimit(m_constants.kMasterConstants.kCurrentLimit);

    m_encoder = m_master.getEncoder();
    m_encoder.setPosition(m_constants.kHomePosition);

    m_slaves = new CANSparkMax[m_constants.kSlaveConstants.length];

    for (int i = 0; i < m_constants.kSlaveConstants.length; i++) {
      m_slaves[i] =
          new CANSparkMax(
              m_constants.kSlaveConstants[i].kID, m_constants.kSlaveConstants[i].kMotorType);
      m_slaves[i].setIdleMode(m_constants.kSlaveConstants[i].kIdleMode);
      m_slaves[i].setSmartCurrentLimit(m_constants.kSlaveConstants[i].kCurrentLimit);
      m_slaves[i].follow(m_master);
    }

    m_pidController = m_master.getPIDController();
    m_pidController.setP(m_constants.kKp, m_constants.kDefaultSlot);
    m_pidController.setI(m_constants.kKi, m_constants.kDefaultSlot);
    m_pidController.setD(m_constants.kKd, m_constants.kDefaultSlot);

    m_profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                m_constants.kMaxVelocity, m_constants.kMaxAcceleration));
  }

  public void runToSetpoint() {
    if (m_previousDesiredState != m_desiredState || m_lastHeldState == m_constants.kManualState) {
      if (m_lastHeldState == m_constants.kManualState) {
        m_lastHeldState = m_constants.kManualState;
      } else {
        m_lastHeldState = m_currentState;
      }
    }

    m_setpoint =
        m_profile.calculate(
            Timer.getFPGATimestamp() - m_profileStartTime,
            new TrapezoidProfile.State(
                m_lastHeldState.getPosition(), m_lastHeldState.getVelocity()),
            new TrapezoidProfile.State(m_desiredState.getPosition(), 0));

    m_pidController.setReference(
        m_setpoint.position,
        ControlType.kPosition,
        m_constants.kDefaultSlot,
        m_arbFeedforward,
        ArbFFUnits.kVoltage);

    if (m_currentState != m_constants.kTransitionState)
      m_currentState = m_constants.kTransitionState;

    m_constants.kTransitionState.setPosition(m_setpoint.position);
    m_constants.kTransitionState.setVelocity(m_setpoint.velocity);

    if (m_setpoint.position == m_desiredState.getPosition()) {
      m_profileStartTime = -1;
      m_lastHeldState = m_desiredState;
      m_currentState = m_desiredState;
      m_constants.kManualState.setPosition(0);
    }

    m_previousDesiredState = m_desiredState;
  }

  public void manualControl(double throttle) {
    double m_throttle = MathUtil.applyDeadband(throttle, m_constants.kManualDeadBand);

    if (m_currentState != m_constants.kManualState)
      m_constants.kManualState.setPosition(getPosition());

    if (Math.abs(m_throttle) > 0 && m_profileStartTime == -1) {
      m_lastHeldState = m_constants.kManualState;
      m_desiredState = m_constants.kManualState;
      m_currentState = m_constants.kManualState;

      m_throttle *= m_constants.kManualMultiplier;

      m_constants.kManualState.setPosition(m_constants.kManualState.getPosition() + m_throttle);
      m_constants.kManualState.setPosition(
          MathUtil.clamp(
              m_constants.kManualState.getPosition(),
              m_constants.kMinPosition,
              m_constants.kMaxPosition));
    }
  }

  public void holdPosition() {
    m_pidController.setReference(
        m_currentState.getPosition(),
        ControlType.kPosition,
        m_constants.kDefaultSlot,
        m_arbFeedforward,
        ArbFFUnits.kVoltage);
  }

  public SubsystemState getCurrentState() {
    return m_currentState;
  }

  public void setFeedforward(double feedforward) {
    m_arbFeedforward = feedforward;
  }

  public void setState(SubsystemState desiredState) {
    m_desiredState = desiredState;
    m_profileStartTime = Timer.getFPGATimestamp();
  }

  public boolean atSetpoint() {
    return Math.abs(m_desiredState.getPosition() - getPosition()) < m_constants.kSetpointTolerance;
  }

  public double getPosition() {
    return RobotBase.isReal() ? m_encoder.getPosition() : m_currentState.getPosition();
  }

  public double getVelocity() {
    return RobotBase.isReal() ? m_encoder.getVelocity() : m_currentState.getVelocity();
  }

  @Override
  public void abstractSubsystemPeriodic() {
    if (m_profileStartTime == -1) {
      holdPosition();
    } else {
      runToSetpoint();
    }
  }

  @Override
  public void outputAbstractSubsystemTelemetry() {
    Logger.recordOutput(
        m_constants.kName + "/Encoder Position", getPosition()); // Current position of encoders
    Logger.recordOutput(m_constants.kName + "/Encoder Velocity", getVelocity()); // Encoder Velocity
    Logger.recordOutput(
        m_constants.kName + "/Trapezoid Desired Position",
        m_currentState.getPosition()); // Desired position of trapezoid profile
    Logger.recordOutput(
        m_constants.kName + "/Trapezoid Desired Velocity",
        m_currentState.getVelocity()); // Desired position of trapezoid profile
    Logger.recordOutput(
        m_constants.kName + "/Desired Position", m_desiredState.getPosition()); // Desired position
    Logger.recordOutput(
        m_constants.kName + "/Last Held Position",
        m_lastHeldState.getPosition()); // Desired position
    Logger.recordOutput(
        m_constants.kName + "/Current State", m_currentState.getName()); // Current State
    Logger.recordOutput(
        m_constants.kName + "/Desired State", m_desiredState.getName()); // Current State
    Logger.recordOutput(m_constants.kName + "/At Setpoint", atSetpoint()); // Is at setpoint
  }
}
