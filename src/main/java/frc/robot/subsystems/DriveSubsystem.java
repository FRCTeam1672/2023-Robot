package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX rightFrontDriveMotor = new WPI_TalonSRX(2);
    private final WPI_TalonSRX leftFrontDriveMotor = new WPI_TalonSRX(3);
    private final WPI_TalonSRX backRightDriveMotor = new WPI_TalonSRX(4);
    private final WPI_TalonSRX backLeftDriveMotor = new WPI_TalonSRX(5);

    private final SlewRateLimiter driveRateLimiter = new SlewRateLimiter(3);

    private final DifferentialDrive drive;

    public DifferentialDrive getDrive() {
        return drive;
    }

    
    private final CommandXboxController xboxController;

    public CommandXboxController getXboxController() {
        return xboxController;
    }

    private boolean isAuto = false;

    private double speed = 0.2;

    public DriveSubsystem(CommandXboxController controller) {
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
    }

    public void changeSpeed(double diff) {
        speed += diff;
        speed = Math.min(speed, 0.3); // TODO: determine good mod
        speed = Math.max(speed, 0);
        //SmartDashboard.putNumber("Drive Speed", 0.65 + speed);
    }

    @Override
    public void periodic() {
        if(DriverStation.isAutonomous() || isAuto) return;


        //grab controller X and Y vales
        //pass to DifferentialDrive arcadedrive (x foward, y rotate)
        double xSpeed = /*-(0.65 + speed) */ -xboxController.getLeftY();
        
        double zRotation = /*-(0.75 + speed / 2) */ -xboxController.getRightX();
        if(Math.abs(xSpeed) <= 0.20 && Math.abs(zRotation) <= 0.2) {
            stop();
            return;
        }
        drive.arcadeDrive(MathUtil.clamp(driveRateLimiter.calculate(xSpeed), -0.85, 0.85), MathUtil.clamp(zRotation, -1, 1
        ));
    }
    public void drive(double xSpeed, double zSpeed) {
        drive.arcadeDrive(xSpeed, -zSpeed);
    }

    public void isAuto(boolean b) {
        isAuto = b;
    }

    public void stop() {
        drive.stopMotor();
        rightFrontDriveMotor.stopMotor();
        leftFrontDriveMotor.stopMotor();
    }
}
