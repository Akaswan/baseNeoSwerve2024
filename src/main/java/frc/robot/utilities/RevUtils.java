package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class RevUtils {

  /**
   * Sets the drive motor configuration
   *
   * @param motorController the motor controller to tune
   */
  // public static void setDriveMotorConfig(CANSparkFlex motorController) {

  //   motorController.getPIDController().setFF(DriveConstants.drivekff);
  //   motorController.getPIDController().setP(DriveConstants.drivekp);
  //   motorController.getPIDController().setI(DriveConstants.driveki);
  //   motorController.getPIDController().setD(DriveConstants.drivekd);

  //   motorController.setOpenLoopRampRate(DriveConstants.driverampRate);

  //   motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
  //   motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
  //   motorController.setPeriodicFramePeriod(
  //       PeriodicFrame.kStatus2, 50); // May want to increase

  //   motorController.setSmartCurrentLimit(60, 35);
  //   motorController.burnFlash();
  // }

  // /**
  //  * Sets the turn motor configuration
  //  *
  //  * @param motorController the motor controller to tune
  //  */
  // public static void setTurnMotorConfig(CANSparkFlex motorController) {

  //   motorController.getPIDController().setFF(DriveConstants.turnkff);
  //   motorController.getPIDController().setP(DriveConstants.turnkp);
  //   motorController.getPIDController().setI(DriveConstants.turnki);
  //   motorController.getPIDController().setD(DriveConstants.turnkd);

  //   motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
  //   motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
  //   motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

  //   motorController.setSmartCurrentLimit(40, 25);
  //   motorController.burnFlash();
  // }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}
