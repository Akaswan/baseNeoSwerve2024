package frc.robot.StateMachine.MechStates;

public enum ArmState implements MechState {
    IN(0),
    LOW(5),
    MID(10),
    HIGH(15);

    private final int position;

    ArmState(int position) {
        this.position = position;
    }

    @Override
    public int getPosition() {
        return position;
    }
}
