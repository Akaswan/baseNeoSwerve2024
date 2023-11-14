package frc.robot.StateMachine;

import frc.robot.StateMachine.MechStates.ArmState;
import frc.robot.StateMachine.MechStates.ShoulderState;

public enum RobotState {
    IN((ArmState.IN), (ShoulderState.IN)),
    LOW((ArmState.LOW), (ShoulderState.LOW)),
    MID_CONE((ArmState.IN), (ShoulderState.IN)),
    MID_CUBE((ArmState.IN), (ShoulderState.IN)),
    HIGH_CONE((ArmState.IN), (ShoulderState.IN)),
    HIGH_CUBE((ArmState.IN), (ShoulderState.IN));

    private final ArmState armState;
    private final ShoulderState shoulderState;

    RobotState(ArmState armState, ShoulderState shoulderState) {
        this.armState = armState;
        this.shoulderState = shoulderState;
    }

    public ArmState getArmState() {
        return armState;
    }

    public ShoulderState getShoulderState() {
        return shoulderState;
    }
}
