package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.SerialPort;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX rightFrontDriveMotor = new WPI_TalonSRX(2);
    private final WPI_TalonSRX leftFrontDriveMotor = new WPI_TalonSRX(3);
    private final WPI_TalonSRX backRightDriveMotor = new WPI_TalonSRX(4);
    private final WPI_TalonSRX backLeftDriveMotor = new WPI_TalonSRX(5);

    private final DifferentialDrive drive;
    private final DifferentialDriveOdometry odometry;

    private Pose2d currentPose;
    private final Field2d field = new Field2d();

    private final CommandXboxController xboxController;
    private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(5);

    public DriveSubsystem(CommandXboxController controller) {
        navX.zeroYaw();
        currentPose = new Pose2d(0, 0, new Rotation2d(0));
        this.rightFrontDriveMotor.setInverted(true);
        this.backRightDriveMotor.setInverted(true);

        this.backLeftDriveMotor.follow(this.leftFrontDriveMotor);
        this.backRightDriveMotor.follow(this.rightFrontDriveMotor);

        this.leftFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
        this.rightFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
        this.backLeftDriveMotor.setNeutralMode(NeutralMode.Brake);
        this.backRightDriveMotor.setNeutralMode(NeutralMode.Brake);

        this.drive = new DifferentialDrive(leftFrontDriveMotor, rightFrontDriveMotor);
        this.xboxController = controller;
        odometry = new DifferentialDriveOdometry(
                navX.getRotation2d(), 0, 0, currentPose);
    }

    public Command getAutoCommand() {
        PathPlannerTrajectory traj = PathPlanner.loadPath("New New New Path", new PathConstraints(4, 3));

        new SequentialCommandGroup(
                new PPRamseteCommand(
                        traj,
                        this::getPose, // Pose supplier
                        new RamseteController(),
                        new SimpleMotorFeedforward(KS, KV, KA),
                        this.kinematics, // DifferentialDriveKinematics
                        this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                        new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0
                                                    // will only use feedforwards.
                        new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                        this::outputVolts, // Voltage biconsumer
                        true, // Should the path be automatically mirrored depending on alliance color.
                            ~  // Optional, defaults to true
                        this // Requires this drive subsystem
                ));
    }

    @Override
    public void periodic() {
        currentPose = odometry.update(navX.getRotation2d(), leftFrontDriveMotor.getSelectedSensorPosition(),
                rightFrontDriveMotor.getSelectedSensorPosition());
        field.setRobotPose(currentPose);
        // grab controller X and Y vales
        // pass to DifferentialDrive arcadedrive (x foward, y rotate)
        double xSpeed = -xboxController.getLeftY();
        double zRotation = -xboxController.getRightX();

        drive.arcadeDrive(MathUtil.clamp(xSpeed, -0.7, 0.7), MathUtil.clamp(zRotation, -0.7, 0.7));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                stepsPerDeciSecToMetersPerSec(leftFrontDriveMotor.getSelectedSensorVelocity()),
                stepsPerDeciSecToMetersPerSec(rightFrontDriveMotor.getSelectedSensorVelocity()));
    }

    private double stepsToMeters(double steps) {
        return (0.154 / 4906) * steps;
    }

    private double stepsPerDeciSecToMetersPerSec(double stepsPerDecisec) {
        return stepsToMeters(stepsPerDecisec * 10);
    }
}
