package frc.robot.commands.elevator.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends CommandBase{
    private ArmSubsystem armSubsystem;

    public IntakeCommand(ArmSubsystem armSubsystem){
        addRequirements(armSubsystem);
        this.armSubsystem = armSubsystem;
    }
    @Override
    public void execute() {
        armSubsystem.intake();
    }
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
    }
    
}
