package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveElevatorDownCommand extends CommandBase{
     private ArmSubsystem elevator;
    public MoveElevatorDownCommand(ArmSubsystem elevator){
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