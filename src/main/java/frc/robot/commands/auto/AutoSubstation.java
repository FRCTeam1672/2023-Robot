package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoSubstation extends CommandBase{
    private VisionSubsystem visionSubsystem;
    private DriveSubsystem drive;

    public AutoSubstation(DriveSubsystem drive, VisionSubsystem visionSubsystem){
        this.drive = drive;
        this.visionSubsystem = visionSubsystem;
    }
    @Override
    public void execute() {
        if(visionSubsystem.getDistanceFromSubstation() == -1 || visionSubsystem.getDistanceFromSubstation() < 2){
            drive.drive(0, 0);
            return;
        }
        drive.drive(0.6, 0);
    }
    
}