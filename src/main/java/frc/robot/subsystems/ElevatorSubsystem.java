package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private final CANSparkMax rElevator = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax lElevator = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax winch = new CANSparkMax(13, MotorType.kBrushless);
    private boolean calibrating = false;

    private final DigitalInput bottomElevatorLimitSwitch = new DigitalInput(9);
    private final DigitalInput winchLimitSwitch = new DigitalInput(8);

    private final ADXRS450_Gyro elevatorGyro = new ADXRS450_Gyro();
    public ElevatorSubsystem(){
        SmartDashboard.putData("Home Winch", Commands.runOnce(() -> {
            if(!calibrating){
                calibrating = true;
                return;
            }
            winch.getEncoder().setPosition(0);
            calibrating = false;
        }, this).ignoringDisable(true));
        lElevator.follow(rElevator, true);
    }
    public void moveDown(){
        // if(elevatorGyro.getAngle() <= 80){
        //     winch.stopMotor();
        //     return;
        // }
        winch.set(-0.5);
    }
    public void moveUp(){
        // if(elevatorGyro.getAngle() >= 10) {
        //     stopWinch();
        //     return;
        // }
        winch.set(0.95);
    }
    public void extend(){
        if(rElevator.getEncoder().getPosition() > 70){
            stopElevators();
            return;
        }
        //extend it a couple of inches 😏
        rElevator.set(0.6);
    }
    public void retract(){
        if(!bottomElevatorLimitSwitch.get()) {
             stopElevators();
             rElevator.getEncoder().setPosition(0);
             return;
        }

        //sheathe it, you heathen!
        rElevator.set(-0.6);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LElevator Encoder: ", lElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("RElevator Encoder: ", rElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Winch Elevator Encoder: ", winch.getEncoder().getPosition());
        SmartDashboard.putBoolean("Calibrating", calibrating);
        SmartDashboard.putBoolean("Winch Limit Switch", winchLimitSwitch.get());
        SmartDashboard.putBoolean("Bottom Elevator Limit Switch", bottomElevatorLimitSwitch.get());
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
}
