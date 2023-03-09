package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GyroUtils;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class BalanceRobot extends CommandBase {
    private final GyroSubsystem gyroSubsystem;
    private final DriveSubsystem driveSubsystem;

    public BalanceRobot(GyroSubsystem gyroSubsystem, DriveSubsystem driveSubsystem) {
        addRequirements(gyroSubsystem);
        this.gyroSubsystem = gyroSubsystem;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        AHRS ahrs = this.gyroSubsystem.getAHRS();
        double xSpeed = GyroUtils.getRoll(ahrs.getPitch());
        this.driveSubsystem.drive(xSpeed, 0.0);
    }
}