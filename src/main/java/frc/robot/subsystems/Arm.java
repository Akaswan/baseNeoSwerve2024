// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateMachine.ArmState;
import frc.robot.StateMachine.StateMachine;

public class Arm extends SubsystemBase implements StateMachine <ArmState>{
  /** Creates a new Arm. */
  private StateMachineInputs<ArmState> inputs = new StateMachineInputs<ArmState>();
  private Map<ArmState, Runnable> stateMap;

  public Arm(ArmState defaultState) {
    inputs.state = defaultState;

    initializeStateMap();
  }

  @Override
  public void initializeStateMap() {
    stateMap = new HashMap<>();

    stateMap.put(ArmState.IN, () -> {
      System.out.println("IN");
    });
    stateMap.put(ArmState.OUT, () -> {
      System.out.println("OUT");
    });
  }

  @Override
  public void handleStateAction() {
    stateMap.get(getState()).run();
  }

  @Override
  public void setState(ArmState state) {
    inputs.previousState = inputs.state;
    inputs.state = state;

    if (inputs.previousState != inputs.state) {
      handleStateAction();
    }
  }

  @Override
  public ArmState getState() {
    return inputs.state;
  }

  @Override
  public void periodic() {
    
  }
}
