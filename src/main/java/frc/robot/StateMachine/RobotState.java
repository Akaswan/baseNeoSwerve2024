package frc.robot.StateMachine;

import frc.robot.StateMachine.MechStates.ArmState;
import frc.robot.StateMachine.MechStates.ShoulderState;

public enum RobotState {
    IN((ArmState.IN), (ShoulderState.IN), ("In")),
    LOW((ArmState.LOW), (ShoulderState.LOW), ("Low")),
    MID_CONE((ArmState.IN), (ShoulderState.IN), ("Mid Cone")),
    MID_CUBE((ArmState.IN), (ShoulderState.IN), ("Mid Cube")),
    HIGH_CONE((ArmState.IN), (ShoulderState.IN), ("High Cone")),
    HIGH_CUBE((ArmState.IN), (ShoulderState.IN), ("High Cube"));

    private final ArmState armState;
    private final ShoulderState shoulderState;
    private final String stringState;

    RobotState(ArmState armState, ShoulderState shoulderState, String stringState) {
        this.armState = armState;
        this.shoulderState = shoulderState;
        this.stringState = stringState;
    }

    public ArmState getArmState() {
        return armState;
    }

    public ShoulderState getShoulderState() {
        return shoulderState;
    }

    public String getStringState() {
        return stringState;
    }
}
