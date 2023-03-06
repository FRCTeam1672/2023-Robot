package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.utils.GyroUtils;

public class BalanceRobot extends CommandBase {
    private GyroSubsystem gyroSubsystem;
    private DriveSubsystem driveSubsystem;
    private int finished = 0;

    public BalanceRobot(GyroSubsystem gyroSubsystem, DriveSubsystem driveSubsystem) {
        addRequirements(gyroSubsystem);
        this.gyroSubsystem = gyroSubsystem;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Balancing", true);
        AHRS ahrs = this.gyroSubsystem.getAHRS();
        double xSpeed = GyroUtils.getRoll(ahrs.getPitch());
        if(xSpeed == 0.0){
            SmartDashboard.putBoolean("Balancing Zero", true);
            finished++;
        }
        else{
            finished = 0;
            SmartDashboard.putBoolean("Balancing Zero", false);
        }
        this.driveSubsystem.drive(xSpeed, 0.0);
    }
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Balancing", false);
        SmartDashboard.putBoolean("Balancing Zero", false);
    }
}
