package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GyroUtils;

public class GyroSubsystem extends SubsystemBase {
    private AHRS ahrs;
    private final ADXRS450_Gyro gryo = new ADXRS450_Gyro();

    public GyroSubsystem() {
        this.ahrs = new AHRS(Port.kMXP);
        System.out.println("is connected?:  " + this.ahrs.isConnected());
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Roll",ahrs.getPitch());
        SmartDashboard.putNumber("Roll Value", GyroUtils.getRoll(ahrs.getPitch()));
    }

    public AHRS getAHRS() {
        return this.ahrs;
    }
}
