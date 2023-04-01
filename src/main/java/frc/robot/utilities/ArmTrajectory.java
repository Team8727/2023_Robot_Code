// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// This file is an adapted version of WPILib's Trajectory class designed for a double jointed arm

package frc.robot.utilities;

import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.kArm.Constraints;
import frc.robot.subsystems.Arm;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class ArmTrajectory {
  private final double totalTime;
  private final List<State> states;

  public ArmTrajectory() {
    totalTime = 0.0;
    states = new ArrayList<>();
  }

  public ArmTrajectory(final List<State> states) {
    this.states = states;
    totalTime = states.get(states.size() - 1).time;
  }

  public ArmTrajectory(Matrix<N4, N1> startState, Matrix<N4, N1> endState) {
    if (!Arm.validPosition(startState.block(2, 1, 0, 0))
        || !Arm.validPosition(endState.block(2, 1, 0, 0))) {
      throw new IllegalArgumentException("Start or End is out of bounds!");
    }

    // Inverse Kinematics to get the Thetas
    Matrix<N2, N1> initialThetas =
        Arm.inverseKinematics(startState.block(2, 1, 0, 0)); // Shoulder, then Elbow
    Matrix<N2, N1> endThetas = Arm.inverseKinematics(endState.block(2, 1, 0, 0));
    // Create the Motion Profiles
    TrapezoidProfile shoulderProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constraints.Proximal.velocity, Constraints.Proximal.acceleration), // contraints
            new TrapezoidProfile.State(endThetas.get(0, 0), endState.get(2, 0)), // endpoint
            new TrapezoidProfile.State(
                initialThetas.get(0, 0), startState.get(2, 0))); // startpoint
    TrapezoidProfile elbowProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constraints.Forearm.velocity, Constraints.Forearm.acceleration), // contraints
            new TrapezoidProfile.State(endThetas.get(1, 0), endState.get(3, 0)), // endpoint
            new TrapezoidProfile.State(
                initialThetas.get(1, 0), startState.get(3, 0))); // startpoint

    totalTime = Math.max(shoulderProfile.totalTime(), elbowProfile.totalTime());
    states = new ArrayList<State>();
    for (double t = 0; t <= totalTime; t += 20.0 / 1000.0) {
      var shoulderState = shoulderProfile.calculate(t);
      var elbowState = elbowProfile.calculate(t);
      states.add(
          new State(
              t,
              new MatBuilder<N4, N1>(Nat.N4(), Nat.N1())
                  .fill(
                      shoulderState.position,
                      elbowState.position,
                      shoulderState.velocity,
                      elbowState.velocity)));
    }
  }

  public static ArmTrajectory linearArmTrajectory(Matrix<N2, N1> start, Matrix<N2, N1> end) {
    if (!Arm.validPosition(start) || !Arm.validPosition(end)) {
      throw new IllegalArgumentException("Start or End is out of bounds!");
    }

    double distance =
        Math.sqrt(
            Math.pow(end.minus(start).get(0, 0), 2) + Math.pow(end.minus(start).get(1, 0), 2));

    var linearPath =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constraints.linearVelocity, Constraints.linearAcceleration),
            new TrapezoidProfile.State(0, 0),
            new TrapezoidProfile.State(distance, 0));

    var states = new ArrayList<State>();
    var prev = start;
    for (double t = 0; t <= linearPath.totalTime(); t += 20.0 / 1000.0) {
      var xy = start.plus(end.minus(start).times(t / linearPath.totalTime()));
      var joints = Arm.inverseKinematics(xy);
      var prevJoints = Arm.inverseKinematics(prev);
      var jointSpeeds = joints.minus(prevJoints).div(20.0 / 1000.0);
      var state = new Matrix<N4, N1>(Nat.N4(), Nat.N1());

      state.assignBlock(0, 0, joints);
      state.assignBlock(2, 0, jointSpeeds);
      prev = xy;

      states.add(new State(t, state));
    }

    return new ArmTrajectory(states);
  }

  private static double lerp(double startValue, double endValue, double t) {
    return startValue + (endValue - startValue) * t;
  }

  private static Matrix<N4, N1> lerp(Matrix<N4, N1> startValue, Matrix<N4, N1> endValue, double t) {
    var dydx = endValue.minus(startValue);
    return dydx.times(t).plus(startValue);
  }

  public Matrix<N4, N1> getInitialPose() {
    return sample(0).state;
  }

  public double getTotalTimeSeconds() {
    return totalTime;
  }

  public List<State> getStates() {
    return states;
  }

  public State sample(double time) {
    if (time <= states.get(0).time) {
      return states.get(0);
    }
    if (time >= totalTime) {
      return states.get(states.size() - 1);
    }

    // To get the element that we want, we will use a binary search algorithm
    // instead of iterating over a for-loop. A binary search is O(std::log(n))
    // whereas searching using a loop is O(n).

    // This starts at 1 because we use the previous state later on for
    // interpolation.
    int low = 1;
    int high = states.size() - 1;

    while (low != high) {
      int mid = (low + high) / 2;
      if (states.get(mid).time < time) {
        // This index and everything under it are less than the requested
        // timestamp. Therefore, we can discard them.
        low = mid + 1;
      } else {
        // t is at least as large as the element at this index. This means that
        // anything after it cannot be what we are looking for.
        high = mid;
      }
    }

    // High and Low should be the same.

    // The sample's timestamp is now greater than or equal to the requested
    // timestamp. If it is greater, we need to interpolate between the
    // previous state and the current state to get the exact state that we
    // want.
    final State sample = states.get(low);
    final State prevSample = states.get(low - 1);

    // If the difference in states is negligible, then we are spot on!
    if (Math.abs(sample.time - prevSample.time) < 1E-9) {
      return sample;
    }
    // Interpolate between the two states for the state that we want.
    return prevSample.interpolate(
        sample, (time - prevSample.time) / (sample.time - prevSample.time));
  }

  public ArmTrajectory concatenate(ArmTrajectory other) {
    // If this is a default constructed trajectory with no states, then we can
    // simply return the rhs trajectory.
    if (states.isEmpty()) {
      return other;
    }

    // Deep copy the current states.
    List<State> new_states =
        states.stream()
            .map(state -> new State(state.time, state.state))
            .collect(Collectors.toList());

    // Here we omit the first state of the other trajectory because we don't want
    // two time points with different states. Sample() will automatically
    // interpolate between the end of this trajectory and the second state of the
    // other trajectory.
    for (int i = 1; i < other.getStates().size(); ++i) {
      var s = other.getStates().get(i);
      new_states.add(new State(s.time + totalTime, s.state));
    }
    return new ArmTrajectory(new_states);
  }

  public ArmTrajectory reverse() {
    if (states.isEmpty()) {
      return this;
    }

    var reverse =
        new MatBuilder<N4, N4>(Nat.N4(), Nat.N4())
            .fill(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1);
    List<State> new_states = new ArrayList<State>();

    for (int i = states.size(); i-- > 0; ) {
      new_states.add(
          new State(this.totalTime - states.get(i).time, reverse.times(states.get(i).state)));
    }

    return new ArmTrajectory(new_states);
  }

  public static class State {
    @JsonProperty("time")
    public double time;

    @JsonProperty("state")
    public Matrix<N4, N1> state;

    public State(double time, Matrix<N4, N1> state) {
      this.time = time;
      this.state = state;
    }

    State interpolate(State endValue, double i) {
      // Find the new t value.
      final double newT = lerp(time, endValue.time, i);

      return new State(newT, lerp(state, endValue.state, i));
    }
  }
}
