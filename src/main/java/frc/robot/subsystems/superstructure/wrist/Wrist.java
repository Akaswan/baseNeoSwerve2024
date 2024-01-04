package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.subsystems.templates.ServoSubsystemSparkMax;
import frc.robot.subsystems.templates.SubsystemConstants.ServoSubsystemSparkMaxConstants;

public class Wrist extends ServoSubsystemSparkMax {

  private ArmFeedforward m_feedforward;

  private static Wrist m_instance = null;

  public Wrist(ServoSubsystemSparkMaxConstants constants) {
    super(constants);

    m_feedforward = new ArmFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);
  }

  public static Wrist getInstance() {
    if (m_instance == null) {
      m_instance = new Wrist(WristConstants.kWristConstants);
    }

    return m_instance;
  }

  @Override
  public void outputTelemetry() {}

  @Override
  public void subsystemPeriodic() {
    setFeedforward(m_feedforward.calculate(m_encoder.getPosition(), m_encoder.getVelocity()));

    Arm.wristLig.setAngle(m_currentState.getPosition());
  }

  public enum WristState implements ServoSubsystemState {
    MANUAL(0, 0, "Manual"),
    TRANSITION(0, 0, "Transition"),
    HOME(155, 0, "Home"),
    GROUND_PICKUP(14, 0, "Ground Pickup"),
    SUBSTATION_PICKUP(-42, 0, "Substation Pickup"),
    SCORE_HIGH(-24, 0, "Score High"),
    SCORE_MID(-24, 0, "Score Mid"),
    SCORE_LOW(-24, 0, "Score Low");

    private double position;
    private double velocity;
    private String name;

    private WristState(double position, double velocity, String name) {
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
