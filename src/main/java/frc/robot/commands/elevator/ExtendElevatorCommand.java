package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendElevatorCommand extends CommandBase{
    private ArmSubsystem elevator;
    public ExtendElevatorCommand(ArmSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
     }
     @Override
     public void execute() {
        elevator.extend();
     }
     @Override
     public void end(boolean interrupted) {
        elevator.stopElevators();
    }
}
