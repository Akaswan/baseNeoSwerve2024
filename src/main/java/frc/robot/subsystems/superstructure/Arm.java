package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.templates.ServoSubsystem;
import frc.robot.subsystems.templates.SubsystemConstants.ServoSubsystemConstants;

public class Arm extends ServoSubsystem {

  private static Arm m_instance = null;

  private ArmFeedforward m_feedforward;
  public static Mechanism2d mech = new Mechanism2d(3, 3);
  public static MechanismRoot2d root = mech.getRoot("SuperStructure", 1.3, .3);
  public static MechanismLigament2d armLig;
  public static MechanismLigament2d wristLig;

  public static synchronized Arm getInstance() {
    if (m_instance == null) {
      m_instance = new Arm(ArmConstants.kArmConstants);
    }

    return m_instance;
  }

  public Arm(ServoSubsystemConstants constants) {
    super(constants);

    m_feedforward = new ArmFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);

    armLig = root.append(new MechanismLigament2d("Arm", 1, 90, 10, new Color8Bit(Color.kPurple)));
    wristLig =
        armLig.append(
            new MechanismLigament2d("Wrist", .33, 90, 7.5, new Color8Bit(Color.kAliceBlue)));
    SmartDashboard.putData("Mech2d", mech);
  }

  @Override
  public void outputTelemetry() {}

  @Override
  public void subsystemPeriodic() {
    setFeedforward(m_feedforward.calculate(m_encoder.getPosition(), m_encoder.getVelocity()));

    armLig.setAngle(m_currentState.getPosition());
  }

  public enum ArmState implements ServoSubsystemState {
    MANUAL(0, 0, "Manual"),
    TRANSITION(0, 0, "Transition"),
    HOME(0, 0, "Home"),
    SUBSTATION_PICKUP(43, 0, "Substation Pickup"),
    SCORE_HIGH(38, 0, "Score High"),
    SCORE_MID(38.5, 0, "Score Mid"),
    SCORE_LOW(15.325, 0, "Score Low");

    private double position;
    private double velocity;
    private String name;

    private ArmState(double position, double velocity, String name) {
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
