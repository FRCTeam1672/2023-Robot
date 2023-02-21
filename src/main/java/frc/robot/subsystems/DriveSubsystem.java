package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX rightFrontDriveMotor = new WPI_TalonSRX(2);
    private final WPI_TalonSRX leftFrontDriveMotor = new WPI_TalonSRX(3);
    private final WPI_TalonSRX backRightDriveMotor = new WPI_TalonSRX(4);
    private final WPI_TalonSRX backLeftDriveMotor = new WPI_TalonSRX(5);

    private final DifferentialDrive drive;

    private final CommandXboxController xboxController;

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
    
    @Override
    public void periodic() {
        //grab controller X and Y vales
        //pass to DifferentialDrive arcadedrive (x foward, y rotate)
        double xSpeed = -xboxController.getLeftY();
        double zRotation = -xboxController.getRightX();
        
        drive.arcadeDrive(MathUtil.clamp(xSpeed, -0.7, 0.7), MathUtil.clamp(zRotation, -0.7, 0.7));
   }
}
