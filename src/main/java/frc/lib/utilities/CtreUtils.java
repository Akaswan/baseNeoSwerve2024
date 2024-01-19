package frc.lib.utilities;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

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
