// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.utilities;

/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as a motor acting
 * against the force of gravity on a beam suspended at an angle).
 */
public class TelescopingArmFeedforward {
  public final double ks;
  public double kg;
  public final double minkg;
  public final double maxkg;
  public final double kv;
  public double ka;
  public final double minka;
  public final double maxka;
  public final double minTelescope;
  public final double maxTelescope;

  /**
   * Creates a new ArmFeedforward with the specified gains. Units of the gain values will dictate
   * units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   */
  public TelescopingArmFeedforward(
      double ks,
      double minkg,
      double maxkg,
      double kv,
      double minka,
      double maxka,
      double minTelescope,
      double maxTelescope) {
    this.ks = ks;
    this.kg = minkg;
    this.minkg = minkg;
    this.maxkg = maxkg;
    this.kv = kv;
    this.ka = minka;
    this.minka = minka;
    this.maxka = maxka;
    this.minTelescope = minTelescope;
    this.maxTelescope = maxTelescope;
  }

  /**
   * Creates a new ArmFeedforward with the specified gains. Acceleration gain is defaulted to zero.
   * Units of the gain values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   */
  public TelescopingArmFeedforward(
      double ks, double minkg, double maxkg, double kv, double minTelescope, double maxTelescope) {
    this(ks, minkg, maxkg, kv, 0, 0, minTelescope, maxTelescope);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param positionRadians The position (angle) setpoint. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param velocityRadPerSec The velocity setpoint.
   * @param accelRadPerSecSquared The acceleration setpoint.
   * @return The computed feedforward.
   */
  public double calculate(
      double positionRadians,
      double velocityRadPerSec,
      double accelRadPerSecSquared,
      double currentTelescope) {
    calculateTelescope(currentTelescope);
    return ks * Math.signum(velocityRadPerSec)
        + kg * Math.cos(positionRadians)
        + kv * velocityRadPerSec
        + ka * accelRadPerSecSquared;
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
   * zero).
   *
   * @param positionRadians The position (angle) setpoint. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double positionRadians, double velocity, double currentTelescope) {
    return calculate(positionRadians, velocity, 0, currentTelescope);
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply, a position, and an
   * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the acceleration constraint, and this will give
   * you a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
   *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
   *     not follow this convention, an offset should be added.
   * @param acceleration The acceleration of the arm.
   * @return The maximum possible velocity at the given acceleration and angle.
   */
  public double maxAchievableVelocity(
      double maxVoltage, double angle, double acceleration, double currentTelescope) {
    // Assume max velocity is positive
    calculateTelescope(currentTelescope);
    return (maxVoltage - ks - Math.cos(angle) * kg - acceleration * ka) / kv;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply, a position, and an
   * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the acceleration constraint, and this will give
   * you a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
   *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
   *     not follow this convention, an offset should be added.
   * @param acceleration The acceleration of the arm.
   * @return The minimum possible velocity at the given acceleration and angle.
   */
  public double minAchievableVelocity(
      double maxVoltage, double angle, double acceleration, double currentTelescope) {
    // Assume min velocity is negative, ks flips sign
    calculateTelescope(currentTelescope);
    return (-maxVoltage + ks - Math.cos(angle) * kg - acceleration * ka) / kv;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply, a position, and
   * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
   *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
   *     not follow this convention, an offset should be added.
   * @param velocity The velocity of the arm.
   * @return The maximum possible acceleration at the given velocity.
   */
  public double maxAchievableAcceleration(
      double maxVoltage, double angle, double velocity, double currentTelescope) {
    calculateTelescope(currentTelescope);
    return (maxVoltage - ks * Math.signum(velocity) - Math.cos(angle) * kg - velocity * kv) / ka;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage supply, a position, and
   * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
   *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
   *     not follow this convention, an offset should be added.
   * @param velocity The velocity of the arm.
   * @return The minimum possible acceleration at the given velocity.
   */
  public double minAchievableAcceleration(
      double maxVoltage, double angle, double velocity, double currentTelescope) {
    return maxAchievableAcceleration(-maxVoltage, angle, velocity, currentTelescope);
  }

  public void calculateTelescope(double currentTelescope) {
    double percentTelescope = currentTelescope / (maxTelescope - minTelescope);
    kg = percentTelescope * (maxkg - minkg) + minkg;
    ka = percentTelescope * (maxka - minka) + minka;
  }
}
