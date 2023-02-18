package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorDownCommand extends CommandBase{
     private ElevatorSubsystem elevator;
    public MoveElevatorDownCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
     }
     @Override
     public void execute() {
        elevator.moveDown();
     }
     @Override
     public void end(boolean interrupted) {
         elevator.stopWinch();
     }
}