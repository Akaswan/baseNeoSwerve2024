package frc.robot.StateMachine;

public interface StateMachine <T> {
    void initializeStateMap();

    void handleStateAction();

    void setState(T state);

    T getState();

    class StateMachineInputs <T> {
        public T state;
        public T previousState;
    }
}
