package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.templates.ServoMotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends ServoMotorSubsystem {

  private ElevatorFeedforward m_feedforward;

  private static Elevator m_instance = null;

  public Elevator(SubsystemConstants constants) {
    super(constants);

    m_feedforward =
        new ElevatorFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);
  }

  public static synchronized Elevator getInstance() {
    if (m_instance == null) {
      m_instance = new Elevator(ElevatorConstants.kElevatorConstants);
    }

    return m_instance;
  }

  @Override
  public void outputSusbsystemTelemetry() {
    Logger.recordOutput(
        "Elevator/2nd Stage Mech3d",
        new Pose3d(
            -.2347
                + MathUtil.clamp(m_currentState.getPosition(), 0, m_constants.kMaxPosition / 2)
                    * Math.cos(Math.toRadians(Arm.getInstance().getPosition())),
            0,
            .254
                + .005
                + MathUtil.clamp(m_currentState.getPosition(), 0, m_constants.kMaxPosition / 2)
                    * Math.sin(Math.toRadians(Arm.getInstance().getPosition())),
            new Rotation3d(
                Math.toRadians(-Arm.getInstance().getPosition() + 90), 0, Math.toRadians(90))));
    Logger.recordOutput(
        "Elevator/3rd Stage Mech3d",
        new Pose3d(
            -.2347
                + m_currentState.getPosition()
                    * Math.cos(Math.toRadians(Arm.getInstance().getPosition())),
            0,
            .254
                + .005
                + m_currentState.getPosition()
                    * Math.sin(Math.toRadians(Arm.getInstance().getPosition())),
            new Rotation3d(
                Math.toRadians(-Arm.getInstance().getPosition() + 90), 0, Math.toRadians(90))));
  }

  @Override
  public void subsystemPeriodic() {
    setFeedforward(m_feedforward.calculate(m_encoder.getPosition(), m_encoder.getVelocity()));

    Arm.root.setPosition(1.5, m_currentState.getPosition());
  }

  public enum ElevatorState implements SubsystemState {
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
