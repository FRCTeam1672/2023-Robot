package frc.robot.commands.elevator.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends CommandBase{
    private IntakeSubsystem intakeSubsystem;

    public OuttakeCommand(IntakeSubsystem intakeSubsystem){
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;
    }
    @Override
    public void execute() {
        intakeSubsystem.outtake();
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
    
}
