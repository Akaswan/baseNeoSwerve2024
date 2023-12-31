package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.templates.ServoSubsystem;
import frc.robot.subsystems.templates.SubsystemConstants.ServoSubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends ServoSubsystem {

  private ArmFeedforward m_feedforward;

  private static Wrist m_instance = null;

  public Wrist(ServoSubsystemConstants constants) {
    super(constants);

    m_feedforward = new ArmFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);
  }

  public static synchronized Wrist getInstance() {
    if (m_instance == null) {
      m_instance = new Wrist(WristConstants.kWristConstants);
    }

    return m_instance;
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput(
        "Wrist/Mech3d",
        new Pose3d(
            -0.2574
                + (Elevator.getInstance().getPosition() + .54)
                    * Math.cos(Math.toRadians(Arm.getInstance().getPosition())),
            0,
            0.2715
                + (Elevator.getInstance().getPosition() + .54)
                    * Math.sin(Math.toRadians(Arm.getInstance().getPosition())),
            new Rotation3d(
                Math.toRadians(-Arm.getInstance().getPosition() - getPosition() + 90 + 155),
                0,
                Math.toRadians(90))));
  }

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
