package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
    private final AHRS gyro;

    //find velocity of the pitch
    private final Timer pitchTimer = new Timer();

    //in degress per second

    private double pitchSpeed = 0.0;
    private double lastPitchAngle;

    public GyroSubsystem() {
        this.gyro = new AHRS(Port.kMXP);
        System.out.println("is connected?:  " + this.gyro.isConnected());
        lastPitchAngle = gyro.getPitch();
        pitchTimer.start();
    }


    public AHRS getAHRS() {
        return this.gyro;
    }

    public float getPitch() {
        return gyro.getPitch();
    }

    public void periodic() {
        /*
        speed = (distance) / (time)
        pitchSpeed = (lastPitchAngle - gyro.getPitch()) / pitchTimer.get();
        lastPitchAngle = gyro.getPitch();
        pitchTimer.reset();

        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Gyro Pitch Speed", pitchSpeed);
        */
    }

    /**
     * @return pitch speed in degress per second
     */
    public double getPitchSpeed() {
        return pitchSpeed;
        //SmartDashboard.putBoolean("Is Connected", ahrs.isConnected());
        //SmartDashboard.putNumber("ACTUAL GYRO ANGLE", ahrs.getPitch());
        //SmartDashboard.putNumber("FAKE GYRO ANGLE", GyroUtils.getRoll(ahrs.getPitch()));
    }
}