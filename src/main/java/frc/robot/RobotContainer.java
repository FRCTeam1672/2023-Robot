// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Targeter.Nodes.*;

import frc.robot.Targeter.Grid;
import frc.robot.commands.elevator.ExtendElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorDownCommand;
import frc.robot.commands.elevator.MoveElevatorUpCommand;
import frc.robot.commands.elevator.RetractElevatorCommand;
import frc.robot.commands.elevator.intake.IntakeCommand;
import frc.robot.commands.elevator.intake.OuttakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem(driveController);
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kCTRE);

  private final Targeter targeter = new Targeter();

  public RobotContainer(){
    bindBindings();
  }
  private void bindBindings(){
    driveController.rightBumper().whileTrue(new MoveElevatorUpCommand(armSubsystem));
    driveController.leftBumper().whileTrue(new MoveElevatorDownCommand(armSubsystem));

    driveController.rightTrigger().whileTrue(new IntakeCommand(armSubsystem));
    driveController.leftTrigger().whileTrue(new OuttakeCommand(armSubsystem));

    driveController.a().whileTrue(new ExtendElevatorCommand(armSubsystem));
    driveController.b().whileTrue(new RetractElevatorCommand(armSubsystem));

    operatorController.start().onTrue(targeter.target(null, HYBRID_LEFT));
    operatorController.back().onTrue(targeter.target(null, HYBRID_CENTER));
    operatorController.leftBumper().onTrue(targeter.target(null, HYBRID_RIGHT));
    operatorController.rightBumper().onTrue(targeter.target(null, MID_LEFT));
    operatorController.a().onTrue(targeter.target(null, MID_CENTER));
    operatorController.b().onTrue(targeter.target(null, MID_RIGHT));
    operatorController.x().onTrue(targeter.target(null, HIGH_LEFT));
    operatorController.y().onTrue(targeter.target(null, HIGH_CENTER));
    operatorController.leftStick().onTrue(targeter.target(null, HIGH_RIGHT));

    operatorController.povLeft().onTrue(targeter.target(Grid.LEFT, null));
    operatorController.povDown().onTrue(targeter.target(Grid.COOP, null));
    operatorController.povRight().onTrue(targeter.target(Grid.RIGHT, null));
  }
}
