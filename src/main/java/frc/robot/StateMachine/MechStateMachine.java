package frc.robot.StateMachine;

import java.util.Map;

import frc.robot.StateMachine.MechStates.MechState;

public interface MechStateMachine {
    void initializeStateMap();

    void handleStateAction();

    void setState(MechState state);

    MechState getState();

    Map<MechState, Runnable> getStateMap();

    double getPosition();

    boolean atSetPoint();

    StateMachineTypes getType();

    class StateMachineInputs {
        public MechState state;
        public MechState previousState;

        public int order;
    }
}
