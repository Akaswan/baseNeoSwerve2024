package frc.robot.utilities;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import java.util.ArrayList;
import java.util.List;

public class GeometryUtils {
  private static final double kEps = 1E-9;

  /**
   * Obtain a new Pose2d from a (constant curvature) velocity. See:
   * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp . Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/b5da3c760b78d598b492e1cc51d8331c2ad50f6a/src/main/java/com/team254/lib/geometry/Pose2d.java
   */
  public static Pose2d exp(final Twist2d delta) {
    double sin_theta = Math.sin(delta.dtheta);
    double cos_theta = Math.cos(delta.dtheta);
    double s, c;
    if (Math.abs(delta.dtheta) < kEps) {
      s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
      c = .5 * delta.dtheta;
    } else {
      s = sin_theta / delta.dtheta;
      c = (1.0 - cos_theta) / delta.dtheta;
    }
    return new Pose2d(
        new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
        new Rotation2d(cos_theta, sin_theta));
  }

  /**
   * Logical inverse of the above. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/b5da3c760b78d598b492e1cc51d8331c2ad50f6a/src/main/java/com/team254/lib/geometry/Pose2d.java
   */
  public static Twist2d log(final Pose2d transform) {
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta =
          -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
    }
    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  public static double[] AKitStates(SwerveModuleState[] states) {
    double[] output = new double[8];
    for (int i = 0; i < states.length; i++) {
      output[i * 2] = states[i].angle.getRadians();
      output[i * 2 + 1] = states[i].speedMetersPerSecond;
    }
    return output;
  }

  public static double[] AKitOdometry(Pose2d pose) {
    double[] output = new double[3];
    output[0] = pose.getX();
    output[1] = pose.getY();
    output[2] = pose.getRotation().getRadians();
    return output;
  }

  public static double[] AKitTrajectory(Trajectory traj) {
    double[] output = new double[traj.getStates().size() * 3];

    for (int i = 0; i < traj.getStates().size() * 3; i += 3) {
      output[i] = traj.getStates().get((i / 3)).poseMeters.getX();
      output[i + 1] = traj.getStates().get((i / 3)).poseMeters.getY();
      output[i + 2] = traj.getStates().get((i / 3)).poseMeters.getRotation().getRadians();
    }

    return output;
  }

  public static double[][] AKitCorner(double[] corners) {
    double[][] output = new double[2][corners.length / 2];

    for (int i = 0; i < corners.length; i += 2) {
      output[0][i / 2] = corners[i];
      output[1][i / 2] = corners[i + 1];
    }

    return output;
  }

  public static Trajectory PPAutoToTraj(String auto) {
    List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(auto);
    List<State> states = new ArrayList<>();

    for (PathPlannerPath path : paths) {
      for (PathPoint point : path.getAllPathPoints()) {
        states.add(
            new State(
                0,
                0,
                0,
                new Pose2d(
                    point.position,
                    point.holonomicRotation != null ? point.holonomicRotation : new Rotation2d(0)),
                0));
      }
    }
    return new Trajectory(states);
  }

  public static Pose2d PoseSpaceToFieldSpace(Pose2d targetPose, Pose2d robotPose) {
    return new Pose2d(
        targetPose.getX() + robotPose.getX(),
        targetPose.getY() + robotPose.getY(),
        new Rotation2d(
            targetPose.getRotation().getRadians() + robotPose.getRotation().getRadians()));
  }

  public static double[] getPoseError(Pose2d pose1, Pose2d pose2) {
    double[] output = new double[3];

    output[0] = pose1.getX() - pose2.getX();
    output[1] = pose1.getY() - pose2.getY();
    output[2] = pose1.getRotation().getRadians() - pose2.getRotation().getRadians();

    return output;
  }
}
