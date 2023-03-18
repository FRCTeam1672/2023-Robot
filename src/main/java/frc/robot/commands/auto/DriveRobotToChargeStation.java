package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        //Trigger t = new Trigger(() -> gyro.getPitch() > -11).debounce(0.25);

        if(gyro.getPitch() > -11){
            drive.drive(-0.85, 0);
        }
        else if(gyro.getPitch() <= -11 ){
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
