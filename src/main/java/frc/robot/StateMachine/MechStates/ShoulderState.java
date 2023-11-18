package frc.robot.StateMachine.MechStates;

import frc.robot.RobotContainer;

public enum ShoulderState implements MechState{
    MANUAL(0),
    IN(0),
    LOW(3),
    MID_CUBE(6),
    MID_CONE(9),
    HIGH_CUBE(12),
    HIGH_CONE(15);

    private final double position;

    ShoulderState(double position) {
        this.position = position;
    }

    @Override
    public double getPosition() {
        return position;
    }
}
