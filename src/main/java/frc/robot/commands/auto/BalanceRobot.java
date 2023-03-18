package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class BalanceRobot extends CommandBase {
    private final GyroSubsystem gyroSubsystem;
    private final DriveSubsystem driveSubsystem;
    private Timer stopTimer = new Timer();

    public BalanceRobot(GyroSubsystem gyroSubsystem, DriveSubsystem driveSubsystem) {
        addRequirements(gyroSubsystem);
        this.gyroSubsystem = gyroSubsystem;
        this.driveSubsystem = driveSubsystem;
    }

    public void initialize() {
        SmartDashboard.putBoolean("Fully Balanced", false);
        stopTimer.reset();
    }

    @Override
    public void execute() {
        if(gyroSubsystem.getPitch() < -6) {
            driveSubsystem.drive(-0.6, 0);
        } else if(gyroSubsystem.getPitch() > 6) {
            driveSubsystem.drive(0.55, 0);
            stopTimer.start();
        } else {
            driveSubsystem.drive(0, 0);
        }
    }

    public boolean isFinished() {
        return stopTimer.hasElapsed(1.5);
    }

    @Override
    public void end(boolean interrupted) {
        //SmartDashboard.putBoolean("Attempting Balancing", false);
        SmartDashboard.putBoolean("Fully Balanced", true);
    }
}
