package frc.robot.StateMachine.MechStates;

public enum ShoulderState implements MechState{
    IN(0),
    LOW(10),
    MID_CUBE(20),
    MID_CONE(30),
    HIGH_CUBE(40),
    HIGH_CONE(50);

    private final int position;

    ShoulderState(int position) {
        this.position = position;
    }

    @Override
    public int getPosition() {
        return position;
    }
}
