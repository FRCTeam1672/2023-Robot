package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DriveRobotToChargeStation extends CommandBase {
    private DriveSubsystem drive;
    private GyroSubsystem gryoSubsystem;
    private final AHRS gryo;
    private boolean isDrivingUp = false;
    private boolean isOnChargeStation = false;

    public DriveRobotToChargeStation(DriveSubsystem drive, GyroSubsystem gryo){
        this.drive = drive;
        this.gryoSubsystem = gryo;
        this.gryo = gryo.getAHRS();
    }
    @Override
    public void initialize() {
        isDrivingUp = false;
        isOnChargeStation = false;
    }
    @Override
    public void execute() {
        System.out.println("Drive Up");
        System.out.println("is on charge station: " + isOnChargeStation );
        SmartDashboard.putBoolean("Driving Up", true);
        if(gryo.getRoll() <= -10){
            isDrivingUp = true;
            drive.drive(0.85, 0);
        }
        if(-8 <= gryo.getRoll() && gryo.getRoll() <= 8 ){
            isDrivingUp = false;
            drive.drive(0.85, 0);
        }
        else if(-55 <= gryo.getRoll() && gryo.getRoll() <= 20 ){
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
