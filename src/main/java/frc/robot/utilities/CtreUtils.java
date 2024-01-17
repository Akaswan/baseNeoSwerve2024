package frc.robot.utilities;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.DriveConstants;

public final class CtreUtils {

  /**
   *
   *
   * <h3>generateCanCoderConfig</h3>
   *
   * Generates and returns the configuration of the sensor.
   *
   * @return - The config of the sensor
   */
  public static CANcoderConfiguration generateCanCoderConfig() {
    CANcoderConfiguration sensorConfig = new CANcoderConfiguration();

    sensorConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

    return sensorConfig;
  }

  
  public static TalonFXConfiguration generateDriveMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.Slot0.kP = DriveConstants.drivekp;
    motorConfig.Slot0.kI = DriveConstants.driveki;
    motorConfig.Slot0.kD = DriveConstants.drivekd;
    motorConfig.Slot0.kS = DriveConstants.driveks;
    motorConfig.Slot0.kV = DriveConstants.drivekv;
    motorConfig.Slot0.kA = DriveConstants.driveka;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = DriveConstants.driverampRate;

    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60;

    motorConfig.Feedback.SensorToMechanismRatio = DriveConstants.kDriveRevToMeters;

    return motorConfig;
  }

  public static TalonFXConfiguration generateTurnMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.Slot0.kP = DriveConstants.turnkp;
    motorConfig.Slot0.kI = DriveConstants.turnki;
    motorConfig.Slot0.kD = DriveConstants.turnkd;
    motorConfig.Slot0.kS = DriveConstants.turnks;
    motorConfig.Slot0.kV = DriveConstants.turnkv;
    motorConfig.Slot0.kA = DriveConstants.turnka;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

    motorConfig.Feedback.SensorToMechanismRatio = 360 / DriveConstants.kTurnGearRatio;

    return motorConfig;
  }

  // public static TalonFXConfiguration generateMastorTalonFXConfig(TalonFXConstants m_constants) {
  //   TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  //   motorConfig.Slot0.GravityType = m_constants.kGravityType;
  //   motorConfig.Slot0.kP = m_constants.kKp;
  //   motorConfig.Slot0.kI = m_constants.kKi;
  //   motorConfig.Slot0.kD = m_constants.kKd;
  //   motorConfig.Slot0.kS = m_constants.kKs;
  //   motorConfig.Slot0.kG = m_constants.kKg;
  //   motorConfig.Slot0.kV = m_constants.kKv;
  //   motorConfig.Slot0.kA = m_constants.kKa;
  //   motorConfig.MotionMagic.MotionMagicAcceleration = m_constants.kMaxAcceleration;
  //   motorConfig.MotionMagic.MotionMagicCruiseVelocity = m_constants.kMaxVelocity;
  //   motorConfig.MotionMagic.MotionMagicJerk = m_constants.kMaxJerk;
  //   motorConfig.MotorOutput.NeutralMode = m_constants.kNuetralMode;

  //   return motorConfig;
  // }

  // public static TalonFXConfiguration generateSlaveTalonFXConfig(TalonFXConstants m_constants) {
  //   TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  //   motorConfig.MotorOutput.NeutralMode = m_constants.kNuetralMode;

  //   return motorConfig;
  // }

  /**
   *
   *
   * <h3>checkCtreError</h3>
   *
   * Checks for a specified error.
   *
   * @param errorCode - Code of error
   * @param message - Desired message if error detected
   */
  // public static void checkCtreError(ErrorCode errorCode, String message) {
  //   if (RobotBase.isReal() && errorCode != ErrorCode.OK) {
  //     DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
  //   }
  // }
}
