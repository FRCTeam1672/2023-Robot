package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoSubstation extends CommandBase{
    private VisionSubsystem visionSubsystem;
    private DriveSubsystem drive;
    public static final  PIDController pidController = new PIDController(0.74, 0.08, 0.2);
    static{
        pidController.setSetpoint(1.5);
    }
    public AutoSubstation(DriveSubsystem drive, VisionSubsystem visionSubsystem){
        this.drive = drive;
        this.visionSubsystem = visionSubsystem;
    }
    @Override
    public void initialize() {
        pidController.reset();
    }
    @Override
    public void execute() {
        drive.isAuto(true);
        if(visionSubsystem.getDistanceFromSubstation() == -1 /*|| visionSubsystem.getDistanceFromSubstation() < 1.3*/){
            drive.stop();
            SmartDashboard.putNumber("PID Calculate", drive.getXboxController().getLeftX());
            return;
        }
        double driveAmount = MathUtil.clamp(-pidController.calculate(visionSubsystem.getDistanceFromSubstation()), -0.5, 0.6);
        SmartDashboard.putNumber("PID Calculate", driveAmount);
        drive.drive(driveAmount, drive.getXboxController().getLeftX());
    }
    @Override
    public void end(boolean interrupted) {
        drive.isAuto(false);
    }
    
}
