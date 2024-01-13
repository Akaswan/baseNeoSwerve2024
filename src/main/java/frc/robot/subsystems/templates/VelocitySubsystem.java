// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.templates;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;
import frc.robot.utilities.LoggedShuffleboardTunableNumber;
import org.littletonrobotics.junction.Logger;

public abstract class VelocitySubsystem extends SubsystemBase {

  public VelocitySubsystemConstants m_constants;

  protected final SparkPIDController m_pidController;

  protected final CANSparkMax m_master;
  protected final CANSparkMax[] m_slaves;
  protected final RelativeEncoder m_encoder;

  protected LoggedShuffleboardTunableNumber m_kp;
  protected LoggedShuffleboardTunableNumber m_ki;
  protected LoggedShuffleboardTunableNumber m_kd;

  protected VelocitySubsystemState m_currentState = null;
  protected VelocitySubsystemState m_desiredState = null;

  protected double m_arbFeedforward = 0;

  protected double simpos = 0;

  protected VelocitySubsystem(final VelocitySubsystemConstants constants) {

    m_constants = constants;

    m_currentState = m_constants.kInitialState;
    m_desiredState = m_constants.kInitialState;

    m_master =
        new CANSparkMax(m_constants.kMasterConstants.kID, m_constants.kMasterConstants.kMotorType);
    m_master.setIdleMode(m_constants.kMasterConstants.kIdleMode);
    m_master.setSmartCurrentLimit(m_constants.kMasterConstants.kCurrentLimit);
    m_master.burnFlash();

    REVPhysicsSim.getInstance().addSparkMax(m_master, DCMotor.getNEO(1));

    m_slaves = new CANSparkMax[m_constants.kSlaveConstants.length];

    for (int i = 0; i < m_constants.kSlaveConstants.length; i++) {
      m_slaves[i] =
          new CANSparkMax(
              m_constants.kSlaveConstants[i].kID, m_constants.kSlaveConstants[i].kMotorType);
      m_slaves[i].setIdleMode(m_constants.kSlaveConstants[i].kIdleMode);
      m_slaves[i].setSmartCurrentLimit(m_constants.kSlaveConstants[i].kCurrentLimit);
      m_slaves[i].follow(m_master);
      m_slaves[i].burnFlash();
    }

    if (m_slaves.length > 0) m_master.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

    m_encoder = m_master.getEncoder();
    m_encoder.setVelocityConversionFactor(m_constants.kVelocityConversionFactor);

    // m_kp =
    //     new LoggedShuffleboardTunableNumber(
    //         m_constants.kName + " p",
    //         m_constants.kKp,
    //         RobotContainer.mechTuningTab,
    //         BuiltInWidgets.kTextView,
    //         Map.of("min", 0),
    //         0,
    //         m_constants.kSubsystemType.ordinal());

    // m_ki =
    //     new LoggedShuffleboardTunableNumber(
    //         m_constants.kName + " i",
    //         m_constants.kKi,
    //         RobotContainer.mechTuningTab,
    //         BuiltInWidgets.kTextView,
    //         Map.of("min", 0),
    //         1,
    //         m_constants.kSubsystemType.ordinal());

    // m_kd =
    //     new LoggedShuffleboardTunableNumber(
    //         m_constants.kName + " d",
    //         m_constants.kKd,
    //         RobotContainer.mechTuningTab,
    //         BuiltInWidgets.kTextView,
    //         Map.of("min", 0),
    //         2,
    //         m_constants.kSubsystemType.ordinal());

    m_pidController = m_master.getPIDController();
    m_pidController.setP(m_constants.kKp, m_constants.kDefaultSlot);
    m_pidController.setI(m_constants.kKi, m_constants.kDefaultSlot);
    m_pidController.setD(m_constants.kKd, m_constants.kDefaultSlot);

    setName(m_constants.kName);
  }

  public void runToSetpoint() {
    m_pidController.setReference(m_desiredState.getVelocity(), ControlType.kVelocity, m_constants.kDefaultSlot, m_arbFeedforward, ArbFFUnits.kVoltage);
  }

  public VelocitySubsystemState getCurrentState() {
    return m_currentState;
  }

  public void setFeedforward(double feedforward) {
    m_arbFeedforward = feedforward;
  }

  public void setDesiredState(VelocitySubsystemState desiredState) {
    m_desiredState = desiredState;
  }

  public boolean atSetpoint() {
    return Math.abs(m_desiredState.getVelocity() - getVelocity()) <= m_constants.kSetpointTolerance;
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public VelocitySubsystemType getSubsystemType() {
    return m_constants.kSubsystemType;
  }

  @Override
  public void periodic() {
    
    runToSetpoint();

    if (atSetpoint()) {
        m_currentState = m_desiredState;
    } else {
        m_currentState = m_constants.kTransitionState;
        m_currentState.setVelocity(getVelocity());
    }

    subsystemPeriodic();

    outputTelemetry();

    // if (Constants.kTuningMode) {
    //   m_pidController.setP(m_kp.get(), m_constants.kDefaultSlot);
    //   m_pidController.setI(m_ki.get(), m_constants.kDefaultSlot);
    //   m_pidController.setD(m_kd.get(), m_constants.kDefaultSlot);
    // }

    Logger.recordOutput(m_constants.kName + "/Encoder Velocity", getVelocity()); // Encoder Velocity
    Logger.recordOutput(
        m_constants.kName + "/Desired Velocity", m_desiredState.getVelocity()); // Desired position
    Logger.recordOutput(
        m_constants.kName + "/Current State", m_currentState.getName()); // Current State
    Logger.recordOutput(
        m_constants.kName + "/Desired State", m_desiredState.getName()); // Current State
    Logger.recordOutput(m_constants.kName + "/At Setpoint", atSetpoint()); // Is at setpoint
  }

  public abstract void subsystemPeriodic();

  public abstract void outputTelemetry();

  public enum VelocitySubsystemType {
    LAUNCHER_FLYWHEEL
  }

  public interface VelocitySubsystemState {
    String getName();

    double getVelocity();

    void setVelocity(double velocity);
  }
}
