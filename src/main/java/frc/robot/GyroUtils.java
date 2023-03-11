package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroUtils {
    private static PIDController rollController = new PIDController(Constants.PIDConstants.Rkp, Constants.PIDConstants.Rki, Constants.PIDConstants.Rkd);
     
    public static double getRoll(double robotRoll) {
        if(robotRoll < -15) return -0.7; 
        if(Math.abs(robotRoll) <= 6){
            System.out.println("robot roll too low!");
            SmartDashboard.putBoolean("Balanced", true);
            return 0.05;
        } 
        SmartDashboard.putBoolean("Balanced", false);
        return -MathUtil.clamp(rollController.calculate(robotRoll),     -0.52, 0.59);
    }
}   