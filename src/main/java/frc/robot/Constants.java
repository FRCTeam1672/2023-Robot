package frc.robot;

public final class Constants {
  public static final class Drive {
    public static final int LEFT_LEADER_ID = 10;
    public static final int RIGHT_LEADER_ID = 11;
    public static final int LEFT_FOLLOWER_ID = 12;
    public static final int RIGHT_FOLLOWER_ID = 13;

    public static final double CURVECADE_TURN_SCALE = 0.5;
    public static final double CURVECADE_THRESHOLD = 0.15; 
  }

  public static final class Arm {
    public static final int ELEVATOR_LEFT_ID = 20;
    public static final int ELEVATOR_RIGHT_ID = 21;
    public static final int INTAKE_FAR_ID = 22;
    public static final int INTAKE_NEAR_ID = 23;
    public static final int WINCH_MOTOR_ID = 24;

    public static final double INTAKE_SPEED = 0.8;

    public static final PIDValues ANGLE_PID = new PIDValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final PIDValues EXTENSION_PID = new PIDValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final double ANGLE_TOLERANCE = 0.0;
    public static final double EXTENSION_TOLERANCE = 0.0;
  }

  public static enum ArmState {
    GROUND(0.0, 0.0),
    STOWED(0.0, 0.0),
    BALANCE(0.0, 0.0),
    MID(0.0, 0.0),
    HIGH(0.0, 0.0),
    SHELF(0.0, 0.0);

    private final double angle;
    private final double extension;

    public double getAngle() { return angle; };
    public double getExtension() { return extension; };

    ArmState(double angle, double extension) {
      this.angle = angle;
      this.extension = extension;
    }
  }
}
