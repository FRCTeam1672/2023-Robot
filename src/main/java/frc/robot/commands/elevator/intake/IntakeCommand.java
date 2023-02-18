package frc.robot.commands.elevator.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem){
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;
    }
    @Override
    public void execute() {
        intakeSubsystem.intake();
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
    
}
