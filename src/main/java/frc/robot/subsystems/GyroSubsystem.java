package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GyroUtils;

public class GyroSubsystem extends SubsystemBase {
    private final AHRS ahrs;

    public GyroSubsystem() {
        this.ahrs = new AHRS(Port.kMXP);
        System.out.println("is connected?:  " + this.ahrs.isConnected());
    }
    
    public AHRS getAHRS() {
        return this.ahrs;
    }

    public void periodic() {
        //SmartDashboard.putBoolean("Is Connected", ahrs.isConnected());
        //SmartDashboard.putNumber("ACTUAL GYRO ANGLE", ahrs.getPitch());
        //SmartDashboard.putNumber("FAKE GYRO ANGLE", GyroUtils.getRoll(ahrs.getPitch()));
    }
}