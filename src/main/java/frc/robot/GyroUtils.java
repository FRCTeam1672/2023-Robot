package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class GyroUtils {
    private static PIDController rollController = new PIDController(Constants.PIDConstants.Rkp, Constants.PIDConstants.Rki, Constants.PIDConstants.Rkd);
     
    public static double getRoll(double robotRoll) {
        if(robotRoll >= 10) return 0.65; 
        if(Math.abs(robotRoll) <= 9){
            System.out.println("robot roll too low!");
            return 0.0;
        } 
        return -MathUtil.clamp(rollController.calculate(robotRoll), -0.50, 0.57);
    }
}   