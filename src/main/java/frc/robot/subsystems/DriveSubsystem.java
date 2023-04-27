    package frc.robot.subsystems;

import static frc.robot.Constants.DriveCharacteristics.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveCharacteristics.ENCODER_DISTANCE_PER_PULSE;
import static frc.robot.Constants.DriveCharacteristics.Ka;
import static frc.robot.Constants.DriveCharacteristics.Ks;
import static frc.robot.Constants.DriveCharacteristics.Kv;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.TalonEncoder;

public class DriveSubsystem extends SubsystemBase {

    final int kCountsPerRev = 4096; // Encoder counts per revolution of the motor shaft.
    final double kSensorGearRatio = 1; // Gear ratio is the ratio between the *encoder* and the wheels. On the AndyMark
                                       // drivetrain, encoders mount 1:1 with the gearbox shaft.
    final double kGearRatio = 10.71; // Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead
                                     // of on the gearbox.
    final double kWheelRadiusInches = 3;
    final int k100msPerSecond = 10;

    private final WPI_TalonSRX rightFrontDriveMotor = new WPI_TalonSRX(2);
    private final WPI_TalonSRX leftFrontDriveMotor = new WPI_TalonSRX(3);
    private final WPI_TalonSRX backRightDriveMotor = new WPI_TalonSRX(4);
    private final WPI_TalonSRX backLeftDriveMotor = new WPI_TalonSRX(5);

    private final DifferentialDrive drive;

    private final Field2d field = new Field2d();

    private final CommandXboxController xboxController;
    private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

    private final DifferentialDriveOdometry odometry;

    private final TalonEncoder rightEncoder = new TalonEncoder(rightFrontDriveMotor);
    private final TalonEncoder leftEncoder = new TalonEncoder(leftFrontDriveMotor);
    private RobotContainer robotContainer;

    public DriveSubsystem(CommandXboxController controller, RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        SmartDashboard.putData("Field", field);
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
        Rotation2d rotation = new Rotation2d(Math.toRadians(navX.getFusedHeading()));

        odometry = new DifferentialDriveOdometry(rotation, 0, 0);
        leftEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);

        leftEncoder.reset();
        rightEncoder.reset();    
    }

    private double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = (double) sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / kSensorGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        return positionMeters;
    }

    @Override
    public void periodic() {
        Rotation2d rotation = new Rotation2d(-Math.toRadians(navX.getFusedHeading()));
        SmartDashboard.putNumber("Left Encoder Distance", leftFrontDriveMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Encoder Distance", rightFrontDriveMotor.getSelectedSensorPosition());
        odometry.update(rotation, nativeUnitsToDistanceMeters(-leftFrontDriveMotor.getSelectedSensorPosition()),
                nativeUnitsToDistanceMeters(-rightFrontDriveMotor.getSelectedSensorPosition()));
        field.setRobotPose(new Pose2d(0, 0, rotation));
        // grab controller X and Y vales
        // pass to DifferentialDrive arcadedrive (x foward, y rotate)
        double xSpeed = -xboxController.getLeftY();
        double zRotation = -xboxController.getRightX();

        SmartDashboard.putNumber("XSpeed Controller", xSpeed);
        SmartDashboard.putNumber("ZSpeed Controller", zRotation);
        if(DriverStation.isAutonomousEnabled() || robotContainer.isInAuto()) return;
        drive.arcadeDrive(MathUtil.clamp(xSpeed, -0.8, 0.8), MathUtil.clamp(zRotation, -0.7, 0.7));
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public void drive(double xSpeed, double zSpeed) {
        drive.arcadeDrive(xSpeed, zSpeed);
    }

    public void driveVolts(double leftVolts, double rightVolts) {
        leftFrontDriveMotor.setVoltage(leftVolts);
        rightFrontDriveMotor.setVoltage(rightVolts);
        SmartDashboard.putNumber("LeftVolts", leftVolts);
        SmartDashboard.putNumber("RightVolts", rightVolts);
        drive.feed();
    }

    public Command getAutoCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        odometry.resetPosition(navX.getRotation2d(), 0, 0, traj.getInitialPose());
                    }
                }),
                new PPRamseteCommand(
                        traj,
                        this::getPose, // Pose supplier
                        new RamseteController(),
                        new SimpleMotorFeedforward(Ks, Kv, Ka),
                        DRIVE_KINEMATICS, // DifferentialDriveKinematics
                        this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                        new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        new PIDController(0.1, 0.05, 0.2), // Right controller (usually the same values as left controller)
                        this::driveVolts, // Voltage biconsumer
                        false, // Should the path be automatically mirrored depending on alliance color.
                              // Optional, defaults to true
                        this // Requires this drive subsystem
                )).handleInterrupt(drive::stopMotor);
        // return Commands.none();
    }
}
