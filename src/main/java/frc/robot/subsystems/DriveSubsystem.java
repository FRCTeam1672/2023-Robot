package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Node;

import static frc.robot.Constants.Drive.*;

public class DriveSubsystem extends SubsystemBase {
  private Supplier<Double> forwardStick, turnStick;

  private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
  private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
  private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(LEFT_FOLLOWER_ID);
  private final WPI_VictorSPX rightFollower = new WPI_VictorSPX(RIGHT_FOLLOWER_ID);

  public DriveSubsystem() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
  }

  public void periodic() {
    if(!DriverStation.isTeleopEnabled()) return;

    WheelSpeeds arcadeSpeeds = WheelSpeeds.fromArcade(forwardStick.get(), turnStick.get() * CURVECADE_TURN_SCALE);
    WheelSpeeds curvatureSpeeds = WheelSpeeds.fromCurvature(forwardStick.get(), turnStick.get());

    double hybridScale = Math.min(Math.abs(forwardStick.get()) / CURVECADE_THRESHOLD, 1);

    WheelSpeeds driveSpeeds = new WheelSpeeds(
        curvatureSpeeds.left * hybridScale + arcadeSpeeds.left * (1 - hybridScale),
        curvatureSpeeds.right * hybridScale + arcadeSpeeds.right * (1 - hybridScale));

    leftLeader.set(driveSpeeds.left);
    rightLeader.set(driveSpeeds.right);
  }

  public void driveVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  public void setDriveSticks(Supplier<Double> forwardStick, Supplier<Double> turnStick) {
    this.forwardStick = forwardStick;
    this.turnStick = turnStick;
  }

  private static class WheelSpeeds {
    public final double left;
    public final double right;

    public WheelSpeeds(double left, double right) {
      this.left = left;
      this.right = right;
    }

    public static WheelSpeeds fromArcade(double baseSpeed, double turnSpeed) {
      return new WheelSpeeds(baseSpeed + turnSpeed, baseSpeed - turnSpeed);
    }

    public static WheelSpeeds fromCurvature(double baseSpeed, double turnSpeed) {
      turnSpeed = Math.abs(baseSpeed) * turnSpeed;
      return new WheelSpeeds(baseSpeed + turnSpeed, baseSpeed - turnSpeed);
    }
  }

  public void getToNode(Node.Position position) {
    // TODO
  }
  public boolean inPosition() {
    return true; // TODO
  }
}
