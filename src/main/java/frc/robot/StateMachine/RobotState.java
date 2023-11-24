package frc.robot.StateMachine;

import frc.robot.StateMachine.MechStates.ArmState;
import frc.robot.StateMachine.MechStates.MechState;
import frc.robot.StateMachine.MechStates.ShoulderState;

public enum RobotState {
  MANUAL((ShoulderState.MANUAL), (ArmState.MANUAL), ("Manual")),
  IN((ShoulderState.IN), (ArmState.IN), ("In")),
  LOW((ShoulderState.LOW), (ArmState.LOW), ("Low")),
  MID_CONE((ShoulderState.MID_CONE), (ArmState.MID), ("Mid Cone")),
  MID_CUBE((ShoulderState.MID_CUBE), (ArmState.MID), ("Mid Cube")),
  HIGH_CONE((ShoulderState.HIGH_CONE), (ArmState.HIGH), ("High Cone")),
  HIGH_CUBE((ShoulderState.HIGH_CUBE), (ArmState.HIGH), ("High Cube"));

  private final ShoulderState shoulderState;
  private final ArmState armState;
  private final String stringState;

  RobotState(ShoulderState shoulderState, ArmState armState, String stringState) {
    this.shoulderState = shoulderState;
    this.armState = armState;
    this.stringState = stringState;
  }

  public ShoulderState getShoulderState() {
    return shoulderState;
  }

  public ArmState getArmState() {
    return armState;
  }

  public String getStringState() {
    return stringState;
  }

  public MechState[] getMechStates() {
    return new MechState[] {shoulderState, armState};
  }
}
