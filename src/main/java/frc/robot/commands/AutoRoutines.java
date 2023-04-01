package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.armState;
import frc.robot.Constants.kAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

public class AutoRoutines {
  private final Drivetrain drivetrain;
  private final Arm arm;
  private final Gripper gripper;

  private final LinkedHashMap<String, Command> routineMap = new LinkedHashMap<String, Command>();
  private final SendableChooser<Command> selector = new SendableChooser<Command>();
  private LinkedHashMap<String, PathPlannerTrajectory> paths = new LinkedHashMap<>();

  // EventMaps
  private HashMap<String, Command> eventMapSubCubeTwoPiece = new HashMap<>();
  private HashMap<String, Command> eventMapFarCubeTwoPiece = new HashMap<>();

  private ArmGripperCommands armGripperCommands;

  public AutoRoutines(
      Drivetrain drivetrain, Arm arm, Gripper gripper, ArmGripperCommands armGripperCommands) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.gripper = gripper;
    this.armGripperCommands = armGripperCommands;

    loadPaths();
    loadRoutines();
    populateSendable();
  }

  /**
   * Get the sendable chooser object
   *
   * @return the sendable chooser object
   */
  public SendableChooser<Command> getChooser() {
    return selector;
  }

  // Auto paths go here
  private void loadRoutines() {
    routineMap.put("No Auto", gripper.intakeCommand());
    routineMap.put(
        "Place Only",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME)));
    routineMap.put("Balance Only", drivetrain.AutoBalanceCommand());
    routineMap.put(
        "Yeet Balance",
        gripper
            .intakeCommand()
            .andThen(arm.gotoState(armState.L3))
            .andThen(gripper.yeet())
            .andThen(arm.gotoState(armState.HOME))
            .andThen(drivetrain.AutoBalanceCommand()));
    routineMap.put(
        "Yeet",
        gripper
            .intakeCommand()
            .andThen(arm.gotoState(armState.L2))
            .andThen(gripper.yeet())
            .andThen(arm.gotoState(armState.HOME)));

    // sub routines -----------------------------------------------------
    routineMap.put(
        "subConeMobility",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.followPath(paths.get("subConeMobility"))));
    routineMap.put(
        "subCubeMobility",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.mobilityAuto()));
    routineMap.put(
        "subConeMobilityandBalance",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.followPath(paths.get("subConeMobilityandBalance")),
            drivetrain.AutoBalanceCommand()));
    routineMap.put(
        "subCubeMobilityandBalance",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.followPath(paths.get("subCubeMobilityandBalance")),
            drivetrain.AutoBalanceCommand()));

    // far routines -----------------------------------------------------
    routineMap.put(
        "farConeMobility",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.mobilityAuto()));
    routineMap.put(
        "farCubeMobility",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.mobilityAuto()));
    routineMap.put(
        "farConeMobilityandBalance",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.followPath(paths.get("farConeMobilityandBalance")),
            drivetrain.AutoBalanceCommand()));
    routineMap.put(
        "farCubeMobilityandBalance",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.followPath(paths.get("farCubeMobilityandBalance")),
            drivetrain.AutoBalanceCommand()));

    // mid routines -----------------------------------------------------
    routineMap.put(
        "midConeandBalance",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.followPath(paths.get("midConeandBalance")),
            drivetrain.AutoBalanceCommand()));
    routineMap.put(
        "midCubeandBalance",
        new SequentialCommandGroup(
            gripper.intakeCommand(),
            arm.gotoState(armState.L3),
            armGripperCommands.placeCommand(),
            arm.gotoState(armState.HOME),
            drivetrain.followPath(paths.get("midCubeandBalance")),
            drivetrain.AutoBalanceCommand()));

    // Two piece
    routineMap.put(
        "subCubeTwoPiece",
        drivetrain.followPathwithEvents(paths.get("subCubeTwoPiece"), eventMapSubCubeTwoPiece));
    routineMap.put(
        "farCubeTwoPiece",
        drivetrain.followPathwithEvents(paths.get("farCubeTwoPiece"), eventMapFarCubeTwoPiece));
  }

  private void loadPaths() {
    // Mobility only
    paths.put(
        "farConeMobility",
        PathPlanner.loadPath(
            "farConeMobility",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));
    paths.put(
        "farCubeMobility",
        PathPlanner.loadPath(
            "farCubeMobility",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));
    paths.put(
        "subConeMobility",
        PathPlanner.loadPath(
            "subConeMobility",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));
    paths.put(
        "subCubeMobility",
        PathPlanner.loadPath(
            "subCubeMobility",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));

    // Mobility + Balance
    paths.put(
        "farConeMobilityandBalance",
        PathPlanner.loadPath(
            "farConeMobilityandBalance",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));
    paths.put(
        "farCubeMobilityandBalance",
        PathPlanner.loadPath(
            "farCubeMobilityandBalance",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));
    paths.put(
        "subConeMobilityandBalance",
        PathPlanner.loadPath(
            "subConeMobilityandBalance",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));
    paths.put(
        "subCubeMobilityandBalance",
        PathPlanner.loadPath(
            "subCubeMobilityandBalance",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));

    // Mid Paths
    paths.put(
        "midConeandBalance",
        PathPlanner.loadPath(
            "midConeandBalance",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));
    paths.put(
        "midCubeandBalance",
        PathPlanner.loadPath(
            "midCubeandBalance",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));

    // Two piece stuff
    paths.put(
        "subCubeTwoPiece",
        PathPlanner.loadPath(
            "subCubeTwoPiece",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));
    eventMapSubCubeTwoPiece.put("eject1", gripper.ejectCommand());
    eventMapSubCubeTwoPiece.put("armdown", arm.gotoState(armState.GROUND));
    eventMapSubCubeTwoPiece.put("intakeDown", gripper.intakeCommand().withTimeout(3));
    eventMapSubCubeTwoPiece.put("armUp", arm.gotoState(armState.HOME));
    eventMapSubCubeTwoPiece.put("eject2", gripper.ejectCommand());

    paths.put(
        "farCubeTwoPiece",
        PathPlanner.loadPath(
            "farCubeTwoPiece",
            new PathConstraints(kAuto.velConstraint, kAuto.accConstraint),
            true));
    eventMapFarCubeTwoPiece.put("eject1", gripper.ejectCommand());
    eventMapFarCubeTwoPiece.put("armdown", arm.gotoState(armState.GROUND));
    eventMapFarCubeTwoPiece.put("intakeDown", gripper.intakeCommand().withTimeout(3));
    eventMapFarCubeTwoPiece.put("armUp", arm.gotoState(armState.HOME));
    eventMapFarCubeTwoPiece.put("eject2", gripper.ejectCommand());
  }

  // Iterate over hashmap and add routines to sendable
  private void populateSendable() {
    selector.setDefaultOption("No Auto", routineMap.get("No Auto"));
    for (Map.Entry<String, Command> entry : routineMap.entrySet()) {
      selector.addOption(entry.getKey(), entry.getValue());
    }
  }
}
