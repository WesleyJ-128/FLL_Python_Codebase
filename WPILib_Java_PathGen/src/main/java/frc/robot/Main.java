package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrajectoryUtil.TrajectorySerializationException;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class Main {

  /**
   * Creates a trajectory from a double[] of elements.
   *
   * @param elements A double[] containing the raw elements of the trajectory.
   * @return A trajectory created from the elements.
   */
  private static Trajectory createTrajectoryFromElements(double[] elements) {
    // Make sure that the elements have the correct length.
    if (elements.length % 7 != 0) {
      throw new TrajectorySerializationException(
          "An error occurred when converting trajectory elements into a trajectory.");
    }

    // Create a list of states from the elements.
    List<Trajectory.State> states = new ArrayList<>();
    for (int i = 0; i < elements.length; i += 7) {
      states.add(
          new Trajectory.State(
              elements[i],
              elements[i + 1],
              elements[i + 2],
              new Pose2d(elements[i + 3], elements[i + 4], new Rotation2d(elements[i + 5])),
              elements[i + 6]));
    }
    return new Trajectory(states);
  }

  /**
   * Returns a double[] of elements from the given trajectory.
   *
   * @param trajectory The trajectory to retrieve raw elements from.
   * @return A double[] of elements from the given trajectory.
   */
  public static double[] getElementsFromTrajectory(Trajectory trajectory) {
    // Create a double[] of elements and fill it from the trajectory states.
    double[] elements = new double[trajectory.getStates().size() * 7];

    for (int i = 0; i < trajectory.getStates().size() * 7; i += 7) {
      var state = trajectory.getStates().get(i / 7);
      elements[i] = state.timeSeconds;
      elements[i + 1] = state.velocityMetersPerSecond;
      elements[i + 2] = state.accelerationMetersPerSecondSq;
      elements[i + 3] = state.poseMeters.getX();
      elements[i + 4] = state.poseMeters.getY();
      elements[i + 5] = state.poseMeters.getRotation().getRadians();
      elements[i + 6] = state.curvatureRadPerMeter;
    }
    return elements;
  }




    public static void main(String[] args) {
      
      DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.1101);
      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(0, 112.06, 5),
        kinematics,
        75);
      TrajectoryConfig config = new TrajectoryConfig(0.6, 1);
      config.setKinematics(kinematics);
      config.addConstraint(autoVoltageConstraint);
      config.setReversed(true);

      Trajectory trajectory = new Trajectory();
      try {
        trajectory = TrajectoryUtil.fromPathweaverJson(Path.of("deploy/paths/Blocks.wpilib.json"));
      } catch (IOException ex) {
        System.out.print("failed");

      //Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(), new Pose2d(new Translation2d(-1, -1), new Rotation2d(0)), config);
      double[] elements = getElementsFromTrajectory(trajectory);
      System.out.println(Arrays.toString(elements));

      }
    }
}
