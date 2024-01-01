// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.templates;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.templates.SubsystemConstants.IntakeSubsystemConstants;

public abstract class IntakeSubsystem extends SubsystemBase {

  public IntakeSubsystemConstants m_constants;

  protected final CANSparkMax m_master;
  protected final CANSparkMax[] m_slaves;
  protected final RelativeEncoder m_encoder;

  protected IntakeSubsystemState m_currentState = null;

  protected IntakeSubsystem(final IntakeSubsystemConstants constants) {

    m_constants = constants;

    m_currentState = m_constants.kInitialState;

    m_master =
        new CANSparkMax(m_constants.kMasterConstants.kID, m_constants.kMasterConstants.kMotorType);
    m_master.setIdleMode(m_constants.kMasterConstants.kIdleMode);
    m_master.setSmartCurrentLimit(m_constants.kMasterConstants.kCurrentLimit);
    m_master.setInverted(m_constants.kMasterConstants.kInverted);
    m_master.burnFlash();

    m_slaves = new CANSparkMax[m_constants.kSlaveConstants.length];

    for (int i = 0; i < m_constants.kSlaveConstants.length; i++) {
      m_slaves[i] =
          new CANSparkMax(
              m_constants.kSlaveConstants[i].kID, m_constants.kSlaveConstants[i].kMotorType);
      m_slaves[i].setIdleMode(m_constants.kSlaveConstants[i].kIdleMode);
      m_slaves[i].setSmartCurrentLimit(m_constants.kSlaveConstants[i].kCurrentLimit);
      m_slaves[i].follow(m_master, m_constants.kSlaveConstants[i].kInverted);
      m_slaves[i].burnFlash();
    }

    m_encoder = m_master.getEncoder();

    setName(m_constants.kName);
  }

  public IntakeSubsystemState getCurrentState() {
    return m_currentState;
  }

  public void setState(IntakeSubsystemState desiredState) {
    m_currentState = desiredState;
    m_master.setVoltage(m_currentState.getVoltage());
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public double getVolts() {
    return RobotBase.isReal() ? m_master.getBusVoltage() : m_currentState.getVoltage();
  }

  public IntakeSubsystemType getSubsystemType() {
    return m_constants.kSubsystemType;
  }

  @Override
  public void periodic() {
    subsystemPeriodic();
    outputTelemetry();
  }

  public abstract void subsystemPeriodic();

  public abstract void outputTelemetry();

  public enum IntakeSubsystemType {
    WRIST_INTAKE
  }

  public interface IntakeSubsystemState {
    String getName();

    double getVoltage();
  }
}
