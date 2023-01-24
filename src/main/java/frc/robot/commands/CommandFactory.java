package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class CommandFactory {
  public static Command win(RobotContainer robotContainer, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
    return Commands
      .parallel(
        Commands.run(() -> driveSubsystem.getToNode(robotContainer.getTargetNode().position)),
        Commands.run(() -> armSubsystem.setState(robotContainer.getTargetNode().heightAsArmState()))
      )
      .until(() -> driveSubsystem.inPosition() && armSubsystem.isStable())
      .andThen(armSubsystem.getOuttakeCommand())
      .finallyDo(e -> {stowCommand(armSubsystem); driveSubsystem.restoreTeleop();});
  }

  public static Command groundIntake(ArmSubsystem armSubsystem) {
    return Commands.run(() -> armSubsystem.setState(ArmState.GROUND))
      .until(() -> armSubsystem.isStable())
      .andThen(armSubsystem.getIntakeCommand())
      .finallyDo(e -> stowCommand(armSubsystem));
  }

  public static Command shelfIntake(ArmSubsystem armSubsystem) {
    return Commands.run(() -> armSubsystem.setState(ArmState.SHELF))
      .until(() -> armSubsystem.isStable())
      .andThen(armSubsystem.getIntakeCommand())
      .finallyDo(e -> stowCommand(armSubsystem));
  }

  private static Command stowCommand(ArmSubsystem armSubsystem) {
    return Commands.runOnce(() -> armSubsystem.setState(ArmState.STOWED));
  }
}
