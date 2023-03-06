package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class GyroUtils {
    private static PIDController rollController = new PIDController(Constants.PIDConstants.Rkp, Constants.PIDConstants.Rki, Constants.PIDConstants.Rkd);
     
    public static double getRoll(double robotRoll) {
        if(-30 <= robotRoll && robotRoll <= -15) return -0.73; 
        if(Math.abs(robotRoll) <= 5) return 0.0;
        return -MathUtil.clamp(rollController.calculate(robotRoll), -0.5, 0.5);
    }
}
