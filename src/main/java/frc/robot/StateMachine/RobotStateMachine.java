// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.StateMachine;

import java.sql.Driver;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StateMachine.MechStates.ArmState;
import frc.robot.StateMachine.MechStates.MechState;
import frc.robot.StateMachine.MechStates.ShoulderState;
import frc.robot.commands.SetMechState;

public class RobotStateMachine extends SubsystemBase {
  /** Creates a new RobotStateMachine. */
  private MechStateMachine[] m_mechStateMachines;
  private Map<RobotState, Runnable> stateMap;

  private RobotState m_previousState;
  private RobotState m_state;

  public RobotStateMachine(MechStateMachine[] mechStateMachines, RobotState initialState) {
    m_mechStateMachines = mechStateMachines;
    m_state = initialState;
    m_previousState = initialState;

    // initializeStateMap();
  }

  // public void initializeStateMap() {
  //   stateMap = new HashMap<>();

  //   stateMap.put(RobotState.IN, () -> {m_mechStateMachines[0].setState(ShoulderState.IN); m_mechStateMachines[1].setState(ArmState.IN);});
  //   stateMap.put(RobotState.LOW, () -> {m_mechStateMachines[0].setState(ShoulderState.LOW); m_mechStateMachines[1].setState(ArmState.LOW);});
  //   // stateMap.put(RobotState.MID_CONE, () -> intendedPosition = ArmState.MID.getPosition());
  //   // stateMap.put(RobotState.HIGH_CONE, () -> intendedPosition = ArmState.HIGH.getPosition());
  // }

  // public void handleStateAction() {
  //   stateMap.get(getState()).run();
  // }

  public RobotState getState() {
    return m_state;
  }

  // public void setRobotState(RobotState state, int[] order) {
  //   m_previousState = m_state;
  //   m_state = state;

  //   SequentialCommandGroup group = new SequentialCommandGroup();

  //   if (m_previousState != m_state) {
  //     for (int i : order) {
  //       group.addCommands(new SetMechState(m_mechStateMachines[i], m_mechStateMachines[i].getState()));
  //     }
  //   }
  // }

  public SequentialCommandGroup setRobotState(RobotState state) {

    if (DriverStation.isEnabled()) {
      m_previousState = m_state;
      m_state = state;
    }



    SequentialCommandGroup group = new SequentialCommandGroup();

    if (m_previousState != m_state) {
      for (MechStateMachine mech : m_mechStateMachines) {
        group.addCommands(new SetMechState(mech, mech.getType() == StateMachineTypes.ARM ? getState().getArmState() : getState().getShoulderState()));
      }
    }

    
    return group;
  }

  @Override
  public void periodic() {
    // System.out.println(m_previousState.getStringState() + " " + m_state.getStringState());
    SmartDashboard.putString("Previous Robot State", m_previousState.getStringState());
    SmartDashboard.putString("Robot State", m_state.getStringState());

    if (DriverStation.isEnabled()) {
      System.out.println("ASKDHKLASJDHLASJDHKLJA");
    }
  }
}
