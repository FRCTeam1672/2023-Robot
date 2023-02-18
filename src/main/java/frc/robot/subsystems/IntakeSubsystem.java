package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax rIntake = new CANSparkMax(21, MotorType.kBrushless);
    private final CANSparkMax lIntake = new CANSparkMax(22, MotorType.kBrushless);
    public IntakeSubsystem(){
        rIntake.follow(lIntake);

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
}
