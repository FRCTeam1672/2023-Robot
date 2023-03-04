// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.elevator.ExtendElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorDownCommand;
import frc.robot.commands.elevator.MoveElevatorUpCommand;
import frc.robot.commands.elevator.RetractElevatorCommand;
import frc.robot.commands.elevator.intake.IntakeCommand;
import frc.robot.commands.elevator.intake.OuttakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDLightSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  CommandXboxController xboxController = new CommandXboxController(0);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(xboxController, this);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kCTRE);
  private final LEDLightSubsystem lightSubsystem = new LEDLightSubsystem();
  public LEDLightSubsystem getLightSubsystem() {
    return lightSubsystem;
  }
  public RobotContainer(){
    bindBindings();
  }
  public void updatePDPVolt() {
    SmartDashboard.putNumber("Greggy Voltage", powerDistribution.getCurrent(7));
  }
  private void bindBindings(){
    xboxController.rightBumper().whileTrue(new MoveElevatorUpCommand(elevatorSubsystem));
    xboxController.leftBumper().whileTrue(new MoveElevatorDownCommand(elevatorSubsystem));

    xboxController.rightTrigger().whileTrue(new IntakeCommand(intakeSubsystem));
    xboxController.leftTrigger().whileTrue(new OuttakeCommand(intakeSubsystem));


    xboxController.a().whileTrue(new ExtendElevatorCommand(elevatorSubsystem));
    xboxController.b().whileTrue(new RetractElevatorCommand(elevatorSubsystem));
  }

    public DriveSubsystem getDriveSubsystem() {
      return this.driveSubsystem;
    }
}
