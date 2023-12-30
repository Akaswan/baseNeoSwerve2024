package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.manager.ServoMotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Arm extends ServoMotorSubsystem {

  private ArmFeedforward m_feedforward;
  public static Mechanism2d mech = new Mechanism2d(3, 3);
  public static MechanismRoot2d root = mech.getRoot("SuperStructure", 1.5, 1.5);
  public static MechanismLigament2d armLig;
  public static MechanismLigament2d wristLig;

  public Arm(SubsystemConstants constants) {
    super(constants);

    m_feedforward = new ArmFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);

    armLig = root.append(new MechanismLigament2d("Arm", 1, 90, 10, new Color8Bit(Color.kPurple)));
    wristLig =
        armLig.append(
            new MechanismLigament2d("Wrist", .33, 90, 7.5, new Color8Bit(Color.kAliceBlue)));
    SmartDashboard.putData("Mech2d", mech);
  }

  @Override
  public void outputSusbsystemTelemetry() {
    Logger.recordOutput(
        "Arm/Mech3d",
        new Pose3d(
            -.2347,
            0,
            .254,
            new Rotation3d(
                Math.toRadians(-m_currentState.getPosition() + 90), 0, Math.toRadians(90))));
  }

  @Override
  public void subsystemPeriodic() {
    setFeedforward(m_feedforward.calculate(m_encoder.getPosition(), m_encoder.getVelocity()));

    armLig.setAngle(m_currentState.getPosition());
  }

  public enum ArmState implements SubsystemState {
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
