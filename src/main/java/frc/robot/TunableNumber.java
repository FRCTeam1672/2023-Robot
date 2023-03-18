package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard. Adapted from Mechanical Advantage, team 6328.
 */
public class TunableNumber {
    private static final String tableKey = "TunableNumbers";
  
    private final String key;
    private double dashboardNumber;
    private final double defaultValue;
  
    /**
     * Create a new TunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public TunableNumber(String dashboardKey, double defaultValue) {
      this.key = tableKey + "/" + dashboardKey;
      this.dashboardNumber = this.defaultValue = defaultValue;
      //SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }
  
    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get() {
      return Constants.tuningMode ? SmartDashboard.getNumber(key, defaultValue) : this.dashboardNumber;
    }
  }