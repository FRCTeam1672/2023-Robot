package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.elevator.MoveElevatorDownCommand;
import frc.robot.commands.elevator.RunForCommand;

public class ArmSubsystem extends SubsystemBase{
    private final CANSparkMax rIntake = new CANSparkMax(21, MotorType.kBrushless);
    private final CANSparkMax lIntake = new CANSparkMax(22, MotorType.kBrushless);

    private final CANSparkMax rElevator = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax lElevator = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax winch = new CANSparkMax(13, MotorType.kBrushless);
    private boolean calibrating = false;

    private final DigitalInput bottomElevatorLimitSwitch = new DigitalInput(9);

    public ArmSubsystem(){
        rIntake.follow(lIntake);
        lElevator.follow(rElevator, true);
        SmartDashboard.putData("Home", Commands.runOnce(() -> {
            if(!calibrating){
                calibrating = true;
                return;
            }
            winch.getEncoder().setPosition(0);
            calibrating = false;
        }, this).ignoringDisable(true));
    }
    public void moveDown(){
        winch.set(-0.5);
    }
    public void moveUp(){
        winch.getEncoder().getPosition();
        if(winch.getEncoder().getPosition() >= -3 && !calibrating) {
            stopWinch();
            return;
        }
        winch.set(0.75);
    }
    public void extend(){
        //extend it a couple of inches 😏
        rElevator.set(0.2);
    }
    public void retract(){
        //if limit switch trigger, *do not move*
        if(bottomElevatorLimitSwitch.get()){
            lElevator.getEncoder().setPosition(0);
            rElevator.getEncoder().setPosition(0);
            stopElevators();
            return;
        }
        //sheathe it, you heathen!
        rElevator.set(-0.2);
    }
    public CANSparkMax getlIntake() {
        return lIntake;
    }
    public CANSparkMax getRIntake() {
        return rIntake;
    }
    public void intake(){
        lIntake.set(-0.7);
    }
    public void outtake(){
        lIntake.set(1);
    }
    public void stop() {
        lIntake.stopMotor();
        rIntake.stopMotor();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LElevator Encoder: ", lElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("RElevator Encoder: ", rElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Winch Elevator Encoder: ", winch.getEncoder().getPosition());
        SmartDashboard.putBoolean("Calibrating", calibrating);
    }

    public CANSparkMax getRightElevator() {
        return rElevator;
    }
    public CANSparkMax getLeftElevator() {
        return lElevator;
    }
    public CANSparkMax getWinch() {
        return winch;
    }
    public void stopElevators() {
        lElevator.stopMotor();
        rElevator.stopMotor();
    }
    public void stopWinch() {
        winch.stopMotor();
    }
    public Command getSubstationIntakeCommand(){
        return Commands
            .parallel(
                Commands.run(this::moveUp, null).until(() -> {return winch.getEncoder().getPosition() >= 500 ? true: false;}),
                Commands.run(this::extend, null).until(() -> {return lElevator.getEncoder().getPosition() >= 500 ? true: false;})
            )
            .andThen(new RunForCommand(this::intake, 3), null)
            .andThen(
                Commands.parallel(
                    Commands.run(this::moveDown, null).until(() -> {return winch.getEncoder().getPosition() <= 0 ? true: false;}),
                    Commands.run(this::retract, null).until(() -> {return lElevator.getEncoder().getPosition() <= 0 ? true: false;})
                )
            );
    }
}
