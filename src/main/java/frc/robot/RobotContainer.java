// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Node.Height;
import frc.robot.Targeter.Grid;
import frc.robot.commands.auto.AutoSubstation;
import frc.robot.commands.auto.BalanceRobot;
import frc.robot.commands.auto.DriveRobotToChargeStation;
import frc.robot.commands.elevator.*;
import frc.robot.commands.elevator.intake.IntakeCommand;
import frc.robot.commands.elevator.intake.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LEDLightSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Targeter.Nodes.*;
import static frc.robot.subsystems.LEDLightSubsystem.LedState.*;

import javax.swing.text.StyleContext.SmallAttributeSet;

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

    private final LEDLightSubsystem ledLightSubsystem = new LEDLightSubsystem();
    private final Targeter targeter = new Targeter();

    private final DriveSubsystem driveSubsystem = new DriveSubsystem(driveController);
    private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final SendableChooser<Command> autos = new SendableChooser<>();
    private final VisionSubsystem vision = new VisionSubsystem();




    public RobotContainer() {
        bindBindings();
        CameraServer.startAutomaticCapture();
        PortForwarder.add(5800, "photonvision.local", 5800);
        autos.setDefaultOption("Mobility", getScoreMobilityAuto());
        autos.addOption("Engage Charge Station", getChargeStationAuto());
        autos.addOption("Dock Charge Station", getDockingAuto());
        SmartDashboard.putData("Select Auto", autos);
    }

    private void bindBindings() {
        driveController.rightBumper().whileTrue(new MoveElevatorUpCommand(armSubsystem));
        driveController.leftBumper().whileTrue(new MoveElevatorDownCommand(armSubsystem));

        // driveController.rightTrigger().onTrue(armSubsystem.getIntakeCommand());
        driveController.rightTrigger().whileTrue(new IntakeCommand(armSubsystem));
        driveController.leftTrigger().whileTrue(new OuttakeCommand(armSubsystem));
        
        driveController.x().whileTrue(new AutoSubstation(driveSubsystem, vision));
        driveController.y().onTrue(armSubsystem.getGamePieceStowCommand());

        driveController.a().whileTrue(new ExtendElevatorCommand(armSubsystem));
        driveController.b().whileTrue(new RetractElevatorCommand(armSubsystem));

       operatorController.back().whileTrue(Commands.run(() -> ledLightSubsystem.setColor(PURPLE))
       .handleInterrupt(() -> ledLightSubsystem.setColor(RAINBOW)));
       operatorController.start().whileTrue(Commands.run(() -> ledLightSubsystem.setColor(YELLOW))
       .handleInterrupt(() -> ledLightSubsystem.setColor(RAINBOW)));

       operatorController.leftStick().onTrue(armSubsystem.getGamePieceStowCommand());
       operatorController.rightStick().onTrue(armSubsystem.getShelfIntakeCommand());
       operatorController.rightBumper().whileTrue(new IntakeCommand(armSubsystem));
       operatorController.leftBumper().whileTrue(new OuttakeCommand(armSubsystem));
       operatorController.y().whileTrue(new MoveElevatorUpCommand(armSubsystem));
       operatorController.a().whileTrue(new MoveElevatorDownCommand(armSubsystem));
       operatorController.b().onTrue(armSubsystem.getScoreCommand(Height.HIGH));
       operatorController.x().onTrue(armSubsystem.getScoreCommand(Height.MID));
       operatorController.povUp().whileTrue(new ExtendElevatorCommand(armSubsystem));
       operatorController.povDown().whileTrue(new RetractElevatorCommand(armSubsystem));
       operatorController.povLeft().onTrue(Commands.runOnce(() -> {
        armSubsystem.stopElevator();
        armSubsystem.stopIntake();
        armSubsystem.stopWinch();
        CommandScheduler.getInstance().cancelAll();
    }));
    }
    public SendableChooser<Command> getAutos() {
        return autos;
    }
    public Command getChargeStationAuto() {
        return armSubsystem
                .getStowCommand()
                .andThen(
                    new DriveRobotToChargeStation(driveSubsystem, gyroSubsystem)
                    .andThen(new TimerCommand(() -> driveSubsystem.drive(-0.785, 0), 0.8))
                );
            

    }
    public Command getDockingAuto() {
        return armSubsystem
                .getStowCommand()
                .andThen(
                    new DriveRobotToChargeStation(driveSubsystem, gyroSubsystem)
                    .andThen(new TimerCommand(() -> driveSubsystem.drive(-0.785, 0), 0.81))
                    .andThen(new TimerCommand(() -> driveSubsystem.drive(0, 0.86), 0.6))
                    .andThen(driveSubsystem::stop)
                );

    }
    public Command getScoreMobilityAuto(){
        return armSubsystem.getStowCommand()
                .andThen(armSubsystem.getScoreCommand(Height.HIGH))
                .andThen(new TimerCommand(armSubsystem::outtake, 2))
                .andThen(Commands.runOnce(armSubsystem::stopIntake))
                .andThen(Commands.parallel(armSubsystem.getStowCommand(), new TimerCommand(() -> driveSubsystem.drive(-0.61, 0), 4.25)))
                .andThen(() -> driveSubsystem.drive(0, 0)
            );
    }
}
