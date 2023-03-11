package frc.robot.commands.auto;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DriveRobotToChargeStation extends CommandBase {
    private final DriveSubsystem drive;
    private final AHRS gyro;
    private boolean isOnChargeStation = false;

    public DriveRobotToChargeStation(DriveSubsystem drive, GyroSubsystem gyro){
        this.drive = drive;
        this.gyro = gyro.getAHRS();
    }
    @Override
    public void initialize() {
        isOnChargeStation = false;
    }
    @Override
    public void execute() {
        if(gyro.getPitch() > -10){
            drive.drive(-0.785, 0);
        }
        else if(gyro.getPitch() <= -10 ){
            isOnChargeStation = true;
        }
        SmartDashboard.putBoolean("Driving up", true);
    }
    @Override
    public boolean isFinished() {
        return isOnChargeStation;
    }
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Driving up", false);
    }
    
}