// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.StateMachine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.SetMechState;

public class RobotStateMachine extends SubsystemBase {
  /** Creates a new RobotStateMachine. */
  private MechStateMachine[] m_mechStateMachines;

  private RobotState m_previousState;
  private RobotState m_state;

  public RobotStateMachine(MechStateMachine[] mechStateMachines, RobotState initialState) {
    m_mechStateMachines = mechStateMachines;
    m_state = initialState;
    m_previousState = initialState;
  }

  public RobotState getState() {
    return m_state;
  }

  public RobotState getPreviousState() {
    return m_previousState;
  }

  public boolean isNewState() {
    return m_previousState == m_state;
  }

  public MechStateMachine[] getMechStateMachines() {
    return m_mechStateMachines;
  }  

  public SequentialCommandGroup setRobotStateCommand(RobotState state) {
    m_previousState = m_state;
    m_state = state;
    int[] order;

    if (m_state.getShoulderState().getPosition() < RobotContainer.m_shoulder.getSimPos()) {
      order = new int[]{1, 0};
    } else {
      order = new int[]{0, 1};
    }
    
    SequentialCommandGroup group = new SequentialCommandGroup();

    if (m_previousState != m_state) {
      for (int i = 0; i < m_mechStateMachines.length; i++) {
        group.addCommands(new SetMechState(m_mechStateMachines[order[i]], getState().getMechStates()[order[i]]));
      }
    }
    
    return group;
  }

  public void setRobotState(RobotState state) {
    m_previousState = m_state;
    m_state = state;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Previous Robot State", m_previousState.getStringState());
    SmartDashboard.putString("Robot State", m_state.getStringState());
  }
}
