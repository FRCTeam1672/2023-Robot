package frc.robot.commands.elevator.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class OuttakeCommand extends CommandBase{
    private ArmSubsystem armSubsystem;

    public OuttakeCommand(ArmSubsystem armSubsystem){
        addRequirements(armSubsystem);
        this.armSubsystem = armSubsystem;
    }
    @Override
    public void execute() {
        armSubsystem.outtake();
    }
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopIntake();
    }
    
}
