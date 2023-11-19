package frc.robot.StateMachine.MechStates;

public enum ArmState implements MechState {
    MANUAL(0),
    IN(0),
    LOW(3.5),
    MID(4),
    HIGH(7);

    private final double position;

    ArmState(double position) {
        this.position = position;
    }

    @Override
    public double getPosition() {
        return position;
    }
}
