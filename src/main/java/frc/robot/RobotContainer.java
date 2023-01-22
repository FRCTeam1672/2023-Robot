package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Node.Height;
import frc.robot.Node.Position;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final Command winCommand = CommandFactory.win(this, driveSubsystem, armSubsystem);
  private final Command groundIntakeCommand = CommandFactory.groundIntake(armSubsystem);
  private final Command shelfIntakeCommand = CommandFactory.shelfIntake(armSubsystem);

  private SendableChooser<Command> autonomousChooser;

  private Node targetNode;

  public RobotContainer() {
    initAutonomousChooser();

    configureButtonBindings();

    initDashboard();
  }

  private void initAutonomousChooser() {
    autonomousChooser = new SendableChooser<>();
    autonomousChooser.setDefaultOption("Option 1", null);
    autonomousChooser.addOption("Option 2", null);
    autonomousChooser.addOption("Option 3", null);
  }

  private void configureButtonBindings() {
    driveSubsystem.setDriveSticks(driverController::getLeftY, driverController::getRightX);

    driverController.rightStick().whileTrue(winCommand);
    driverController.leftBumper().whileTrue(groundIntakeCommand);
    driverController.rightBumper().whileTrue(shelfIntakeCommand);

    operatorController.leftBumper().onTrue(moveTargetNode(0, -1));
    operatorController.rightBumper().onTrue(moveTargetNode(0, 1));
    operatorController.leftTrigger(0.8).debounce(0.25).onTrue(moveTargetNode(-1, 0));
    operatorController.rightTrigger(0.8).debounce(0.25).onTrue(moveTargetNode(1, 0));
  }

  public void initDashboard() {
    SmartDashboard.putString("Target Node", "null");
  }

  public void scheduleAutonomousCommand() {
    autonomousChooser.getSelected().schedule();
  }

  public Node getTargetNode() {
    return targetNode;
  }
  public void setTargetNode(Height height, Position position) {
    targetNode = new Node(height, position);
    SmartDashboard.putString("Target Node", targetNode.toString());
  }

  public Command moveTargetNode(int dy, int dx) {
    return Commands.run(() -> {
      int height = clamp(targetNode.height.ordinal()+dy, 0, 2);
      int position = clamp(targetNode.position.ordinal()+dx, 0, 8);

      setTargetNode(Height.values()[height], Position.values()[position]);
    });
  }

  private int clamp(int value, int min, int max) {
    return Math.min(max, Math.max(min, value));
  }
}
