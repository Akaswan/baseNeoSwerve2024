// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.templates;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.templates.SubsystemConstants.ServoSubsystemTalonFXConstants;
// import frc.robot.utilities.CtreUtils;
// import frc.robot.utilities.LoggedShuffleboardTunableNumber;
// import java.util.Map;
// import org.littletonrobotics.junction.Logger;

// public abstract class ServoSubsystemTalonFX extends SubsystemBase {

//   public ServoSubsystemTalonFXConstants m_constants;

//   protected final TalonFX m_master;
//   protected final TalonFX[] m_slaves;

//   protected LoggedShuffleboardTunableNumber m_kp;
//   protected LoggedShuffleboardTunableNumber m_ki;
//   protected LoggedShuffleboardTunableNumber m_kd;
//   protected LoggedShuffleboardTunableNumber m_kMaxAcceleration;
//   protected LoggedShuffleboardTunableNumber m_kMaxVelocity;

//   protected ServoSubsystemState m_currentState = null;
//   protected ServoSubsystemState m_desiredState = null;

//   protected ServoSubsystemTalonFX(final ServoSubsystemTalonFXConstants constants) {

//     m_constants = constants;

//     m_currentState = m_constants.kInitialState;
//     m_desiredState = m_constants.kInitialState;

//     m_master = new TalonFX(m_constants.kMasterConstants.kID, "rio");
//     m_master.getConfigurator().apply(new TalonFXConfiguration());
//     m_master
//         .getConfigurator()
//         .apply(CtreUtils.generateMastorTalonFXConfig(m_constants.kMasterConstants));

//     m_slaves = new TalonFX[m_constants.kSlaveConstants.length];

//     for (int i = 0; i < m_constants.kSlaveConstants.length; i++) {
//       m_slaves[i] = new TalonFX(m_constants.kSlaveConstants[i].kID, "rio");
//       m_slaves[i].getConfigurator().apply(new TalonFXConfiguration());
//       m_slaves[i].setControl(new Follower(m_master.getDeviceID(), false));
//       m_slaves[i]
//           .getConfigurator()
//           .apply(CtreUtils.generateSlaveTalonFXConfig(m_constants.kSlaveConstants[i]));
//     }

//     // if (m_slaves.length > 0) m_master.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

//     m_kp =
//         new LoggedShuffleboardTunableNumber(
//             m_constants.kName + " p",
//             m_constants.kMasterConstants.kKp,
//             RobotContainer.mechTuningTab,
//             BuiltInWidgets.kTextView,
//             Map.of("min", 0),
//             0,
//             m_constants.kSubsystemType.ordinal());

//     m_ki =
//         new LoggedShuffleboardTunableNumber(
//             m_constants.kName + " i",
//             m_constants.kMasterConstants.kKi,
//             RobotContainer.mechTuningTab,
//             BuiltInWidgets.kTextView,
//             Map.of("min", 0),
//             1,
//             m_constants.kSubsystemType.ordinal());

//     m_kd =
//         new LoggedShuffleboardTunableNumber(
//             m_constants.kName + " d",
//             m_constants.kMasterConstants.kKd,
//             RobotContainer.mechTuningTab,
//             BuiltInWidgets.kTextView,
//             Map.of("min", 0),
//             2,
//             m_constants.kSubsystemType.ordinal());

//     m_kMaxAcceleration =
//         new LoggedShuffleboardTunableNumber(
//             m_constants.kName + " Max Acceleration",
//             m_constants.kMasterConstants.kMaxAcceleration,
//             RobotContainer.mechTuningTab,
//             BuiltInWidgets.kTextView,
//             Map.of("min", 0),
//             3,
//             m_constants.kSubsystemType.ordinal());

//     m_kMaxVelocity =
//         new LoggedShuffleboardTunableNumber(
//             m_constants.kName + " Max Velocity",
//             m_constants.kMasterConstants.kMaxVelocity,
//             RobotContainer.mechTuningTab,
//             BuiltInWidgets.kTextView,
//             Map.of("min", 0),
//             4,
//             m_constants.kSubsystemType.ordinal());

//     setName(m_constants.kName);
//   }

//   public void runToSetpoint() {}

//   public void manualControl(double throttle) {
//     double m_throttle = MathUtil.applyDeadband(throttle, m_constants.kManualDeadBand);

//     if (m_currentState != m_constants.kManualState)
//       m_constants.kManualState.setPosition(getPosition());

//     if (Math.abs(m_throttle) > 0
//     // && m_profileStartTime == -1
//     ) {
//       m_desiredState = m_constants.kManualState;
//       m_currentState = m_constants.kManualState;

//       m_throttle *= m_constants.kManualMultiplier;

//       m_constants.kManualState.setPosition(m_constants.kManualState.getPosition() + m_throttle);
//       m_constants.kManualState.setPosition(
//           MathUtil.clamp(
//               m_constants.kManualState.getPosition(),
//               m_constants.kMinPosition,
//               m_constants.kMaxPosition));
//     }
//   }

//   public void holdPosition() {
//     m_master.setControl(new PositionVoltage(m_currentState.getPosition()));
//   }

//   public ServoSubsystemState getCurrentState() {
//     return m_currentState;
//   }

//   // public void setFeedforward(double feedforward) {
//   //   m_arbFeedforward = feedforward;
//   // }

//   public void setState(ServoSubsystemState desiredState) {
//     m_desiredState = desiredState;
//     m_master.setControl(new MotionMagicVoltage(desiredState.getPosition()));
//   }

//   public boolean atSetpoint() {
//     return Math.abs(m_desiredState.getPosition() - getPosition()) <
// m_constants.kSetpointTolerance;
//   }

//   public double getPosition() {
//     return RobotBase.isReal() ? m_master.getPosition().getValue() : m_currentState.getPosition();
//   }

//   public double getVelocity() {
//     return RobotBase.isReal() ? m_master.getVelocity().getValue() : m_currentState.getVelocity();
//   }

//   // public TalonFXServoSubsystemType getSubsystemType() {
//   //   return m_constants.kSubsystemType;
//   // }

//   @Override
//   public void periodic() {
//     // if (m_profileStartTime == -1) {
//     //   holdPosition();
//     // } else {
//     //   runToSetpoint();
//     // }

//     // System.out.println(m_master.getClosedLoopFeedForward().getValue());
//     System.out.println(m_master.getSimState().getMotorVoltage());

//     subsystemPeriodic();

//     outputTelemetry();

//     // if (Constants.kTuningMode) {
//     //   m_pidController.setP(m_kp.get(), m_constants.kDefaultSlot);
//     //   m_pidController.setI(m_ki.get(), m_constants.kDefaultSlot);
//     //   m_pidController.setD(m_kd.get(), m_constants.kDefaultSlot);
//     //   m_profile =
//     //       new TrapezoidProfile(
//     //           new TrapezoidProfile.Constraints(m_kMaxVelocity.get(),
// m_kMaxAcceleration.get()));
//     // }

//     Logger.recordOutput(
//         m_constants.kName + "/Encoder Position", getPosition()); // Current position of encoders
//     Logger.recordOutput(m_constants.kName + "/Encoder Velocity", getVelocity()); // Encoder
// Velocity
//     Logger.recordOutput(
//         m_constants.kName + "/Trapezoid Desired Position",
//         m_currentState.getPosition()); // Desired position of trapezoid profile
//     Logger.recordOutput(
//         m_constants.kName + "/Trapezoid Desired Velocity",
//         m_currentState.getVelocity()); // Desired position of trapezoid profile
//     Logger.recordOutput(
//         m_constants.kName + "/Desired Position", m_desiredState.getPosition()); // Desired
// position
//     Logger.recordOutput(
//         m_constants.kName + "/Current State", m_currentState.getName()); // Current State
//     Logger.recordOutput(
//         m_constants.kName + "/Desired State", m_desiredState.getName()); // Current State
//     Logger.recordOutput(m_constants.kName + "/At Setpoint", atSetpoint()); // Is at setpoint
//   }

//   public abstract void subsystemPeriodic();

//   public abstract void outputTelemetry();

//   public enum ServoSubsystemType {
//     ARM,
//     ELEVATOR,
//     WRIST
//   }

//   public interface ServoSubsystemState {
//     String getName();

//     double getPosition();

//     double getVelocity();

//     void setPosition(double position);

//     void setVelocity(double velocity);
//   }
// }
