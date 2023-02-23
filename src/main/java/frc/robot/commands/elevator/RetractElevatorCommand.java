package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class RetractElevatorCommand extends CommandBase{
    private ArmSubsystem elevator;
    public RetractElevatorCommand(ArmSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
     }
     @Override
     public void execute() {
        elevator.retract();
     }
     @Override
     public void end(boolean interrupted) {
         elevator.stopElevator();
     }
}
