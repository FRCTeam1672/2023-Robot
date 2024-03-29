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
import frc.robot.commands.auto.AutoSubstation;
import frc.robot.commands.auto.DriveRobotToChargeStation;
import frc.robot.commands.elevator.*;
import frc.robot.commands.elevator.intake.IntakeCommand;
import frc.robot.commands.elevator.intake.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LEDLightSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.subsystems.LEDLightSubsystem.LedState.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final LEDLightSubsystem ledLightSubsystem = new LEDLightSubsystem();

    private final DriveSubsystem driveSubsystem = new DriveSubsystem(driveController);
    private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    public ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }

    private final SendableChooser<Command> autos = new SendableChooser<>();
    private final VisionSubsystem vision = new VisionSubsystem();

    public RobotContainer() {
        bindBindings();
        CameraServer.startAutomaticCapture();
        PortForwarder.add(5800, "photonvision.local", 5800);
        autos.setDefaultOption("High Mobility", getScoreMobilityAuto());
        autos.setDefaultOption("Mid Mobility", getScoreMidMobilityAuto());
        // autos.addOption("Engage Charge Station", getChargeStationAuto());
        autos.addOption("Dock Charge Station", getDockingAuto());
        autos.addOption("New Auto Charge Station", getNewChargeStationAuto());
        SmartDashboard.putData("Select Auto", autos);

        SmartDashboard.putData("Frame Perimeter", armSubsystem.getFramePerimeterCommand().asProxy());

        SmartDashboard.putNumber("Auto Drive Backward Speed", -0.745);
        SmartDashboard.putNumber("Auto Drive Backward Duration", 0.825);
        SmartDashboard.putNumber("Auto Dock Speed", 0.86);
        SmartDashboard.putNumber("Auto Dock Duration", 0.485);
    }

    private void bindBindings() {
        driveController.rightBumper().whileTrue(new MoveElevatorUpCommand(armSubsystem));
        driveController.leftBumper().whileTrue(new MoveElevatorDownCommand(armSubsystem));

        // driveController.rightTrigger().onTrue(armSubsystem.getIntakeCommand());
        driveController.rightTrigger().whileTrue(new IntakeCommand(armSubsystem));
        driveController.leftTrigger().whileTrue(new OuttakeCommand(armSubsystem));

        driveController.x().whileTrue(new AutoSubstation(driveSubsystem, vision));
        driveController.y().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            armSubsystem.getGamePieceStowCommand().schedule();
        }));

        driveController.a().whileTrue(new ExtendElevatorCommand(armSubsystem));
        driveController.b().whileTrue(new RetractElevatorCommand(armSubsystem));

        operatorController.back().whileTrue(Commands.run(() -> ledLightSubsystem.setColor(PURPLE))
                .handleInterrupt(() -> ledLightSubsystem.setColor(RAINBOW)));
        operatorController.start().whileTrue(Commands.run(() -> ledLightSubsystem.setColor(YELLOW))
                .handleInterrupt(() -> ledLightSubsystem.setColor(RAINBOW)));

        operatorController.leftStick().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            armSubsystem.getGamePieceStowCommand().schedule();
        }));
        operatorController.rightStick().onTrue(Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            armSubsystem.getShelfIntakeCommand().schedule();
        }));

        operatorController.rightBumper().whileTrue(new IntakeCommand(armSubsystem));
        operatorController.leftBumper().whileTrue(new OuttakeCommand(armSubsystem));
        operatorController.y().whileTrue(new MoveElevatorUpCommand(armSubsystem));
        operatorController.a().whileTrue(new MoveElevatorDownCommand(armSubsystem));
        operatorController.b().onTrue(Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            armSubsystem.getScoreCommand(Height.HIGH).schedule();
        }));
        operatorController.x().onTrue(Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            armSubsystem.getScoreCommand(Height.MID).schedule();

        }));
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

    public Command getNewChargeStationAuto() {
        return armSubsystem
                .getStowCommand()
                .andThen(armSubsystem.getScoreCommand(Height.MID))
                .andThen(new TimerCommand(armSubsystem::outtake, 0.7))
                .andThen(Commands.runOnce(armSubsystem::stopIntake))
                .andThen(armSubsystem.getStowCommand())
                .andThen(
                    Commands.run(() -> driveSubsystem.drive(-0.70, 0))
                    .until(() -> Math.abs(gyroSubsystem.getPitch()) >= 11)
                )            
                .andThen(
                    Commands.run(() -> driveSubsystem.drive(-0.6, 0))
                    .until(() -> Math.abs(gyroSubsystem.getPitch()) <= 8)
                )
                .andThen(new TimerCommand(
                        () -> driveSubsystem.drive(0, SmartDashboard.getNumber("Auto Dock Speed", 0.865)),
                        SmartDashboard.getNumber("Auto Dock Duration", 0.48)))
                .andThen(driveSubsystem::stop);

    }

    public Command getDockingAuto() {

        return armSubsystem
                .getStowCommand()
                .andThen(armSubsystem.getScoreCommand(Height.MID))
                .andThen(new TimerCommand(armSubsystem::outtake, 0.7))
                .andThen(Commands.runOnce(armSubsystem::stopIntake))
                .andThen(armSubsystem.getStowCommand())
                .andThen(
                        new DriveRobotToChargeStation(driveSubsystem, gyroSubsystem)
                                .andThen(new TimerCommand(
                                        () -> driveSubsystem.drive(
                                                SmartDashboard.getNumber("Auto Drive Backward Speed", -0.785), 0),
                                        SmartDashboard.getNumber("Auto Drive Backward Duration", 1.15)))
                                .andThen(new TimerCommand(
                                        () -> driveSubsystem.drive(0,
                                                SmartDashboard.getNumber("Auto Dock Speed", 0.865)),
                                        SmartDashboard.getNumber("Auto Dock Duration", 0.48)))
                                .andThen(driveSubsystem::stop));

    }

    public Command getScoreMobilityAuto() {
        return armSubsystem.getStowCommand()
                .andThen(armSubsystem.getScoreCommand(Height.HIGH))
                .andThen(new TimerCommand(armSubsystem::outtake, 2))
                .andThen(Commands.runOnce(armSubsystem::stopIntake))
                .andThen(Commands.parallel(armSubsystem.getStowCommand(),
                        new TimerCommand(() -> driveSubsystem.drive(-0.61, 0), 4.25)))
                .andThen(() -> driveSubsystem.drive(0, 0));
    }

    public Command getScoreMidMobilityAuto() {
        return armSubsystem.getStowCommand()
                .andThen(armSubsystem.getScoreCommand(Height.MID))
                .andThen(new TimerCommand(armSubsystem::outtake, 2))
                .andThen(Commands.runOnce(armSubsystem::stopIntake))
                .andThen(Commands.parallel(armSubsystem.getStowCommand(),
                        new TimerCommand(() -> driveSubsystem.drive(-0.61, 0), 4.25)))
                .andThen(() -> driveSubsystem.drive(0, 0));
    }
}
