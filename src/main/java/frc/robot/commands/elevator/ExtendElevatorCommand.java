package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ExtendElevatorCommand extends CommandBase{
    private ElevatorSubsystem elevator;
    public ExtendElevatorCommand(ElevatorSubsystem elevator){
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
