package frc.robot.subsystems.templates;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class SuperstructureSubsystem extends SubsystemBase {

  protected SuperstructureState m_currentState;
  protected SuperstructureState m_desiredState;

  protected String m_name;

  public SuperstructureSubsystem(SuperstructureState initialState, String name) {
    m_currentState = initialState;
    m_desiredState = initialState;
    m_name = name;
  }

  public void setDesiredState(SuperstructureState desiredState) {
    m_desiredState = desiredState;
  }

  public void setCurrentState(SuperstructureState currentState) {
    m_currentState = currentState;
  }

  public SuperstructureState getDesiredState() {
    return m_desiredState;
  }

  public SuperstructureState getCurrentState() {
    return m_currentState;
  }

  public abstract SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState);
  
  public abstract SuperstructureState getTransitionState();

  public interface SuperstructureState {
    String getName();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(m_name + " Superstructure/Current State", m_currentState.getName());
    Logger.recordOutput(m_name + " Superstructure/Desired State", m_desiredState.getName());
  }
}
