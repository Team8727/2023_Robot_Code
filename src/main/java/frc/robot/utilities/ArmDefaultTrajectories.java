package frc.robot.utilities;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.armState;
import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class ArmDefaultTrajectories {
  public HashMap<String, ArmTrajectory> trajectories = new HashMap<>();

  public ArmDefaultTrajectories() {
    try {
      JSONParser parser = new JSONParser();
      JSONObject jsonTrajectories =
          (JSONObject)
              parser.parse(
                  new FileReader(
                      new File(Filesystem.getDeployDirectory(), "arm_trajectories_blazing.json")));
      for (Object item : jsonTrajectories.keySet()) {
        JSONObject trajectory = (JSONObject) jsonTrajectories.get(item);

        JSONArray states = (JSONArray) trajectory.get("states");

        trajectories.put((String) item, new ArmTrajectory(toStateList(states)));
      }
    } catch (Exception e) {
      System.out.println(e);
    }
  }

  private List<ArmTrajectory.State> toStateList(JSONArray statesArray) {
    List<ArmTrajectory.State> states = new ArrayList<ArmTrajectory.State>();
    for (Object item : statesArray) {
      JSONObject jsonState = (JSONObject) item;
      JSONArray jsonStateVector = (JSONArray) jsonState.get("state");
      Matrix<N4, N1> stateVector =
          new MatBuilder<N4, N1>(Nat.N4(), Nat.N1())
              .fill(
                  (double) jsonStateVector.get(0),
                  (double) jsonStateVector.get(1),
                  (double) jsonStateVector.get(2),
                  (double) jsonStateVector.get(3));

      var state = new ArmTrajectory.State((double) jsonState.get("time"), stateVector);
      states.add(state);
    }

    return states;
  }

  public ArmTrajectory getTrajectory(Pair<armState, armState> trajPair) {
    return trajectories.get((trajPair.getFirst().name()) + "_" + trajPair.getSecond().name());
  }

  public boolean validTrajectory(Pair<armState, armState> trajPair) {
    return this.getTrajectory(trajPair) != null;
  }
}
