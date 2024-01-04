package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.templates.ServoSubsystemSparkMax;
import frc.robot.subsystems.templates.SubsystemConstants.ServoSubsystemSparkMaxConstants;

public class Elevator extends ServoSubsystemSparkMax {

  private ElevatorFeedforward m_feedforward;

  private static Elevator m_instance = null;

  public Elevator(ServoSubsystemSparkMaxConstants constants) {
    super(constants);

    m_feedforward =
        new ElevatorFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);
  }

  public static Elevator getInstance() {
    if (m_instance == null) {
      m_instance = new Elevator(ElevatorConstants.kElevatorConstants);
    }

    return m_instance;
  }

  @Override
  public void outputTelemetry() {}

  @Override
  public void subsystemPeriodic() {
    setFeedforward(m_feedforward.calculate(m_encoder.getPosition(), m_encoder.getVelocity()));

    Arm.armLig.setLength(m_currentState.getPosition() + .5);
  }

  public enum ElevatorState implements ServoSubsystemState {
    MANUAL(0, 0, "Manual"),
    TRANSITION(0, 0, "Transition"),
    HOME(0, 0, "Home"),
    SUBSTATION_PICKUP(.98, 0, "Substation Pickup"),
    SCORE_HIGH(1, 0, "Score High"),
    SCORE_MID(.516, 0, "Score Mid"),
    SCORE_LOW(.137, 0, "Score Low");

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
