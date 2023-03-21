package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceRobot extends CommandBase {
    private ChiefBalance balancer;
    private DriveSubsystem drive;

    public BalanceRobot(DriveSubsystem drive) {
        this.balancer = new ChiefBalance();
        this.drive = drive;
    }

    public void execute() {
        drive.drive(balancer.autoBalanceRoutine(), 0);
    }

    public boolean isFinished(boolean interrupted) {
        return false;
    }
}
