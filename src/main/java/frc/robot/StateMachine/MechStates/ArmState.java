package frc.robot.StateMachine.MechStates;

import frc.robot.RobotContainer;

public enum ArmState implements MechState {
    MANUAL(0),
    IN(0),
    LOW(5),
    MID(10),
    HIGH(15);

    private final double position;

    ArmState(double position) {
        this.position = position;
    }

    @Override
    public double getPosition() {
        return position;
    }
}
