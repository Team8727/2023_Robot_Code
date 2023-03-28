package frc.robot.utilities;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.robot.Constants.armState;
import java.util.HashMap;

public class ArmDefaultTrajectories {
  public HashMap<String, ArmTrajectory> trajectories = new HashMap<>();

  public ArmDefaultTrajectories() {

    trajectories.put("INIT_HOME", simpleProfile(.07, 0.08, 0.18, 0.16));
    trajectories.put("HOME_INIT", trajectories.get("INIT_HOME").reverse());

    var start = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.18, .16, 0, 0);
    var mid = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.45, .16, -0.75, 0.25);
    var end = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.65, -.08, 0, 0);
    trajectories.put(
        "HOME_GROUND", complexProfile(start, mid).concatenate(complexProfile(mid, end)));

    trajectories.put("GROUND_HOME", trajectories.get("HOME_GROUND").reverse());

    start = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(1.16, 1.24, 0, 0);
    mid = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.65, .9, 0.7, -1.1);
    end = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.18, .16, 0, 0);
    trajectories.put("L3_HOME", complexProfile(start, mid).concatenate(complexProfile(mid, end)));
    trajectories.put("HOME_L3", trajectories.get("L3_HOME").reverse());

    trajectories.put("L2_HOME", simpleProfile(.9, .9, .18, .16));
    trajectories.put("HOME_L2", trajectories.get("L2_HOME").reverse());

    trajectories.put("HOME_DOUBLESUB", simpleProfile(.18, 0.16, 0.65, 0.89));
    trajectories.put("DOUBLESUB_HOME", trajectories.get("HOME_DOUBLESUB").reverse());

    // Neutral trajectories

    trajectories.put("HOME_NEUTRAL", simpleProfile(.17, .16, .49, .49));
    trajectories.put("NEUTRAL_HOME", simpleProfile(.49, 0.49, 0.18, 0.16));

    trajectories.put("L3_NEUTRAL", simpleProfile(1.16, 1.24, .49, .49));
    trajectories.put("NEUTRAL_L3", trajectories.get("L3_NEUTRAL").reverse());

    trajectories.put("NEUTRAL_L2", simpleProfile(.49, .49, .9, .9));
    trajectories.put("L2_NEUTRAL", simpleProfile(.9, .9, .49, .49));

    trajectories.put("NEUTRAL_GROUND", simpleProfile(.49, .49, .65, -.08));
    trajectories.put("GROUND_NEUTRAL", simpleProfile(.65, -.08, .49, .49));

    trajectories.put("NEUTRAL_DOUBLESUB", simpleProfile(.49, .49, .65, 0.89));
    trajectories.put("DOUBLESUB_NEUTRAL", simpleProfile(.89, .9, .49, .49));
  }

  public ArmTrajectory simpleProfile(double startx, double starty, double endx, double endy) {
    return new ArmTrajectory(
        new MatBuilder<N4, N1>(Nat.N4(), Nat.N1()).fill(startx, starty, 0, 0),
        new MatBuilder<N4, N1>(Nat.N4(), Nat.N1()).fill(endx, endy, 0, 0));
  }

  public ArmTrajectory complexProfile(Matrix<N4, N1> start, Matrix<N4, N1> end) {
    return new ArmTrajectory(start, end);
  }

  public ArmTrajectory getTrajectory(Pair<armState, armState> trajPair) {
    return trajectories.get((trajPair.getFirst().name()) + "_" + trajPair.getSecond().name());
  }
}
