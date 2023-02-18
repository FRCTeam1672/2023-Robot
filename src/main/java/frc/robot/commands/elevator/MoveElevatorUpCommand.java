package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorUpCommand extends CommandBase{
     private ElevatorSubsystem elevator;
    public MoveElevatorUpCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
     }
     @Override
     public void execute() {
         elevator.moveUp();
     }
     @Override
     public void end(boolean interrupted) {
         elevator.stopWinch();
     }
}