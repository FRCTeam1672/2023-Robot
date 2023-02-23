package frc.robot.subsystems;

import java.beans.Encoder;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive.DrivePID;
import frc.robot.devices.TalonEncoder;
import frc.robot.Node;

import static frc.robot.Constants.Drive.*;

public class DriveSubsystem extends SubsystemBase {
  private Supplier<Double> forwardStick, turnStick;
  private boolean suppressTeleop = true;

  private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
  private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

  private final VisionSubsystem visionSubsystem;
  private final Gyro gyro = new ADXRS450_Gyro();
  private final TalonEncoder leftEncoder = new TalonEncoder(leftLeader, 6.0 * Math.PI);
  private final TalonEncoder rightEncoder = new TalonEncoder(rightLeader, 6.0 * Math.PI);
  private final DifferentialDriveOdometry odometry;

  public DriveSubsystem(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    this.odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);
  }

  @Override
  public void periodic() {
    m_odometry.update(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    if(suppressTeleop || !DriverStation.isTeleopEnabled()) return;

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

  public void restoreTeleop() {
    suppressTeleop = false;
  }

  public void getToNode(Node node) {
    suppressTeleop = true;
    
    Command movementCommand = getMovementCommand();

    CommandScheduler.getInstance().schedule(movementCommand);
  }
  public boolean inPosition() {
    return true; // TODO
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLeader.getSelectedSensorVelocity() * DrivePID.kEncoderConversion,
                                            rightLeader.getSelectedSensorVelocity() * DrivePID.kEncoderConversion);
  }

  private Command getMovementCommand(Pose2d robotRelativeTargetPose) {
      Trajectory followTrajectory =
          TrajectoryGenerator.generateTrajectory(
              new Pose2d(0, 0, new Rotation2d(0)),
              List.of(),
              robotRelativeTargetPose,
              DrivePID.TRAJECTORY_CONFIG);
  
      RamseteCommand ramseteCommand =
          new RamseteCommand(
              followTrajectory,
              m_robotDrive::getPose,
              new RamseteController(DrivePID.kRamseteB, DrivePID.kRamseteZeta),
              new SimpleMotorFeedforward(
                  DrivePID.ksVolts,
                  DrivePID.kvVoltSecondsPerMeter,
                  DrivePID.kaVoltSecondsSquaredPerMeter),
                  DrivePID.kDriveKinematics,
              m_robotDrive::getWheelSpeeds,
              new PIDController(DrivePID.kPDriveVel, 0, 0),
              new PIDController(DrivePID.kPDriveVel, 0, 0),
              // RamseteCommand passes volts to the callback
              this::driveVoltage,
              this);
  
      resetOdometry(followTrajectory.getInitialPose());
  
      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> driveVoltage(0, 0));
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
}
