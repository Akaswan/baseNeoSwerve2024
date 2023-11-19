package frc.robot.StateMachine.MechStates;

public enum ShoulderState implements MechState{
    MANUAL(0),
    IN(0),
    LOW(-1.5),
    MID_CUBE(6),
    MID_CONE(3),
    HIGH_CUBE(12),
    HIGH_CONE(5.5);

    private final double position;

    ShoulderState(double position) {
        this.position = position;
    }

    @Override
    public double getPosition() {
        return position;
    }
}
