package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DriveRobotToChargeStation extends CommandBase {
    private final DriveSubsystem drive;
    private final AHRS gyro;
    private boolean isDrivingUp = false;
    private boolean isOnChargeStation = false;

    public DriveRobotToChargeStation(DriveSubsystem drive, GyroSubsystem gyro){
        this.drive = drive;
        this.gyro = gyro.getAHRS();
    }
    @Override
    public void initialize() {
        isDrivingUp = false;
        isOnChargeStation = false;
    }
    @Override
    public void execute() {
        System.out.println("is on charge station: " + isOnChargeStation );
        SmartDashboard.putBoolean("Driving Up", true);
        SmartDashboard.putBoolean("Driving Up Var", isDrivingUp);
        if(gyro.getPitch() >= 20){
            isDrivingUp = true;
            drive.drive(0.70, 0);
        }
        if(-8 <= gyro.getPitch() && gyro.getPitch() <= 8 ){
            isDrivingUp = false;
            drive.drive(0.70, 0);
        }
        else if(-55 <= gyro.getPitch() && gyro.getPitch() <= 20 ){
            isOnChargeStation = true;
        }
    }
    @Override
    public boolean isFinished() {
        return isOnChargeStation;
    }
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Driving Up", false);
    }
    
}
