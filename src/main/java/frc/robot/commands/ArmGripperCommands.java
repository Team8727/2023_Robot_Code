package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.armState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import java.util.function.BooleanSupplier;

// Compositions using both the arm and gripper
public class ArmGripperCommands {
  private Arm arm;
  private Gripper gripper;
  // private Indications indications;

  public ArmGripperCommands(Arm arm, Gripper gripper) {
    this.arm = arm;
    this.gripper = gripper;
  }

  public Command placeCommand() {
    return arm.place()
        .andThen(gripper.ejectCommand())
        .andThen(arm.gotoState(armState.HOME))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command intakeCommand(BooleanSupplier gamePieceSelector) {
    return new ConditionalCommand(
        gripper.intakeCommand(GamePiece.CONE),
        gripper.intakeCommand(GamePiece.KUBE),
        gamePieceSelector);
  }
}
