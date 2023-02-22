package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunForCommand extends CommandBase{
    private final double time;
    private final Timer timer = new Timer();
    private final Runnable command;
    public RunForCommand(Runnable command, double time){
        this.command = command;
        this.time = time;
    }
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    @Override
    public void execute(){
        command.run();
    }
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }
}
