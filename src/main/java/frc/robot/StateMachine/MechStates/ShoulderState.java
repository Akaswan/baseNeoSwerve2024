package frc.robot.StateMachine.MechStates;

public enum ShoulderState implements MechState{
    IN(0),
    LOW(3),
    MID_CUBE(6),
    MID_CONE(9),
    HIGH_CUBE(12),
    HIGH_CONE(15);

    private final int position;

    ShoulderState(int position) {
        this.position = position;
    }

    @Override
    public int getPosition() {
        return position;
    }
}
