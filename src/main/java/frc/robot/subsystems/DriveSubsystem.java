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

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.TalonEncoder;
import frc.robot.subsystems.LEDLightSubsystem.LedState;

public class DriveSubsystem extends SubsystemBase {

    final int kCountsPerRev = 4096; // Encoder counts per revolution of the motor shaft.
    final double kSensorGearRatio = 1; // Gear ratio is the ratio between the *encoder* and the wheels. On the AndyMark
                                       // drivetrain, encoders mount 1:1 with the gearbox shaft.
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

    /* start simulation */
    private final EncoderSim sim_leftEncoderSim = new EncoderSim(new Encoder(0, 1));
    private final EncoderSim sim_rightEncoderSim = new EncoderSim(new Encoder(2, 3));
    private final SimDouble sim_gyro = new SimDouble(
            SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

    private final DifferentialDrivetrainSim sim_drive = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDualCIMPerSide,
            KitbotGearing.k8p45,
            KitbotWheelSize.kSixInch,
            null);

    /* end simulation */
    private final RobotContainer robotContainer;

    public DriveSubsystem(CommandXboxController controller, RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        navX.calibrate();
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

        odometry = new DifferentialDriveOdometry(navX.getRotation2d(), 0, 0);
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
        /* start simulation */
        field.setRobotPose(getPose());
        /* end simulation */

        // grab controller X and Y vales
        // pass to DifferentialDrive arcadedrive (x foward, y rotate)
        double xSpeed = -xboxController.getLeftY();
        double zRotation = xboxController.getRightX();

        drive.arcadeDrive(MathUtil.clamp(xSpeed, -0.83, 0.83), MathUtil.clamp(zRotation, -0.7, 0.7));
    }

    /* Start simulation */
    @Override
    public void simulationPeriodic() {
        sim_drive.setInputs(rightFrontDriveMotor.get() * RobotController.getInputVoltage(),
                leftFrontDriveMotor.get() * RobotController.getInputVoltage());

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        sim_drive.update(0.02);

        // Update all of our sensors.
        sim_leftEncoderSim.setDistance(sim_drive.getLeftPositionMeters());
        sim_leftEncoderSim.setRate(sim_drive.getLeftVelocityMetersPerSecond());
        sim_rightEncoderSim.setDistance(sim_drive.getRightPositionMeters());
        sim_rightEncoderSim.setRate(sim_drive.getRightVelocityMetersPerSecond());
        sim_gyro.set(-sim_drive.getHeading().getDegrees());
        odometry.resetPosition(sim_drive.getHeading(), 0, 0, sim_drive.getPose());
    }
    /* End simulation */

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
        drive.feed();
    }

    public Command getAutoCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        odometry.resetPosition(navX.getRotation2d(), 0, 0, traj.getInitialPose());
                        rightFrontDriveMotor.setSelectedSensorPosition(0);
                        leftFrontDriveMotor.setSelectedSensorPosition(0);
                    }
                }),
                new PPRamseteCommand(
                        traj,
                        this::getPose, // Pose supplier
                        new RamseteController(),
                        new SimpleMotorFeedforward(Ks, Kv, Ka),
                        DRIVE_KINEMATICS, // DifferentialDriveKinematics
                        this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                        new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0
                                                    // will only use feedforwards.
                        new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                        this::driveVolts, // Voltage biconsumer
                        true, // Should the path be automatically mirrored depending on alliance color.
                              // Optional, defaults to true
                        this // Requires this drive subsystem
                ),
                new InstantCommand(
                        () -> {
                            robotContainer.getLightSubsystem().setState(LedState.YELLOW);
                        })

        );
    }
}
