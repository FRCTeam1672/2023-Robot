package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class RetractElevatorCommand extends CommandBase{
    private ElevatorSubsystem elevator;
    public RetractElevatorCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
     }
     @Override
     public void execute() {
        elevator.retract();
     }
     @Override
     public void end(boolean interrupted) {
         elevator.stopElevators();
     }
}
