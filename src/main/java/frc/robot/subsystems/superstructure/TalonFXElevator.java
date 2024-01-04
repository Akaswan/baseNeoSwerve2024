// package frc.robot.subsystems.superstructure;

// import frc.robot.Constants.TalonFXElevatorConstants;
// import frc.robot.subsystems.templates.ServoSubsystemTalonFX;
// import frc.robot.subsystems.templates.ServoSubsystemTalonFX.ServoSubsystemState;
// import frc.robot.subsystems.templates.SubsystemConstants.ServoSubsystemTalonFXConstants;

// public class TalonFXElevator extends ServoSubsystemTalonFX {

//   private static TalonFXElevator m_instance = null;

//   public TalonFXElevator(ServoSubsystemTalonFXConstants constants) {
//     super(constants);
//   }

//   public static synchronized TalonFXElevator getInstance() {
//     if (m_instance == null) {
//       m_instance = new TalonFXElevator(TalonFXElevatorConstants.kTalonFXElevatorConstants);
//     }

//     return m_instance;
//   }

//   @Override
//   public void outputTelemetry() {}

//   @Override
//   public void subsystemPeriodic() {}

//   public enum TalonFXElevatorState implements ServoSubsystemState {
//     MANUAL(0, 0, "Manual"),
//     TRANSITION(0, 0, "Transition"),
//     HOME(0, 0, "Home"),
//     SUBSTATION_PICKUP(100, 0, "Substation Pickup"),
//     SCORE_HIGH(200, 0, "Score High"),
//     SCORE_MID(300, 0, "Score Mid"),
//     SCORE_LOW(245, 0, "Score Low");

//     private double position;
//     private double velocity;
//     private String name;

//     private TalonFXElevatorState(double position, double velocity, String name) {
//       this.position = position;
//       this.velocity = velocity;
//       this.name = name;
//     }

//     @Override
//     public double getPosition() {
//       return position;
//     }

//     @Override
//     public double getVelocity() {
//       return velocity;
//     }

//     @Override
//     public void setPosition(double position) {
//       this.position = position;
//     }

//     @Override
//     public void setVelocity(double velocity) {
//       this.velocity = velocity;
//     }

//     @Override
//     public String getName() {
//       return name;
//     }
//   }
// }
