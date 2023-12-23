package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.subsystems.manager.ServoMotorSubsystem;

public class Elevator extends ServoMotorSubsystem {

  private ElevatorFeedforward m_feedforward;

  public Elevator(SubsystemConstants constants) {
    super(constants);

    m_feedforward =
        new ElevatorFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);
  }

  @Override
  public void outputTelemetry() {}

  @Override
  public void lowLevelSubsystemPeriodic() {
    setFeedforward(m_feedforward.calculate(m_encoder.getPosition(), m_encoder.getVelocity()));

    Arm.root.setPosition(1.5, m_currentState.getPosition());
  }

  public enum ElevatorState implements SubsystemState {
    MANUAL(0, 0, "Manual"),
    TRANSITION(0, 0, "Transition"),
    SETPOINT_SWITCH(0, 0, "Setpoint Switch"),
    HOME(0, 0, "Home"),
    OUT(1.5, 0, "Out"),
    IN(.5, 0, "In");

    private double position;
    private double velocity;
    private String name;

    private ElevatorState(double position, double velocity, String name) {
      this.position = position;
      this.velocity = velocity;
      this.name = name;
    }

    @Override
    public double getPosition() {
      return position;
    }

    @Override
    public double getVelocity() {
      return velocity;
    }

    @Override
    public void setPosition(double position) {
      this.position = position;
    }

    @Override
    public void setVelocity(double velocity) {
      this.velocity = velocity;
    }

    @Override
    public String getName() {
      return name;
    }
  }
}
