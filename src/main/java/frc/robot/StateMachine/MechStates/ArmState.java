package frc.robot.StateMachine.MechStates;

public enum ArmState implements MechState {
    IN(0),
    LOW(20),
    MID(30),
    HIGH(40);

    private final int position;

    ArmState(int position) {
        this.position = position;
    }

    @Override
    public int getPosition() {
        return position;
    }
}
