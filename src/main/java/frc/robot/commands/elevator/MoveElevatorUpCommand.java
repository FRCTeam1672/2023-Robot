package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveElevatorUpCommand extends CommandBase{
     private ArmSubsystem elevator;
    public MoveElevatorUpCommand(ArmSubsystem elevator){
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