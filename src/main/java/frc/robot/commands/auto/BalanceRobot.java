package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
                                              /*              |  It controls how fast the robot needs to be*/
                                              /* CHANGE THIS  v  moving before sending the stop (idle) command*/
        double driveSpeed = gyroSubsystem.getPitchSpeed() >= -5 ? -0.7 : 0;
        SmartDashboard.putBoolean("Fully Balanced", driveSpeed == 0);
        driveSubsystem.drive(driveSpeed, 0.0);
    }
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Attempting Balancing", false);
        SmartDashboard.putBoolean("Fully Balanced", false);
    }
}
