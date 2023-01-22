package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmState;

import static frc.robot.Constants.Arm.*;

public class ArmSubsystem extends SubsystemBase {
  private ArmState armState;

  private final CANSparkMax elevatorLeftMotor = new CANSparkMax(ELEVATOR_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax elevatorRightMotor = new CANSparkMax(ELEVATOR_RIGHT_ID, MotorType.kBrushless);
  private final CANSparkMax intakeFarMotor = new CANSparkMax(INTAKE_FAR_ID, MotorType.kBrushless);
  private final CANSparkMax intakeNearMotor = new CANSparkMax(INTAKE_NEAR_ID, MotorType.kBrushless);
  private final CANSparkMax elevatorWinchMotor = new CANSparkMax(WINCH_MOTOR_ID, MotorType.kBrushless);

  private final SparkMaxPIDController elevatorLeftController = elevatorLeftMotor.getPIDController();
  private final SparkMaxPIDController elevatorRightController = elevatorRightMotor.getPIDController();
  private final SparkMaxPIDController elevatorWinchController = elevatorWinchMotor.getPIDController();

  public ArmSubsystem() {
    elevatorRightMotor.setInverted(true);
    elevatorRightMotor.getEncoder().setInverted(true);

    intakeNearMotor.follow(intakeFarMotor);
    intakeNearMotor.setInverted(true);

    initPIDControllers();
  }

  public void initPIDControllers() {
    
  }

  public void setState(ArmState newState) {
    armState = newState;
  }

  public boolean isStable() {
    return Math.abs(elevatorLeftMotor.getEncoder().getPosition() - armState.getExtension()) < EXTENSION_TOLERANCE &&
        Math.abs(elevatorRightMotor.getEncoder().getPosition() - armState.getExtension()) < EXTENSION_TOLERANCE &&
        Math.abs(elevatorWinchMotor.getEncoder().getPosition() - armState.getAngle()) < ANGLE_TOLERANCE;
  }

  @Override
  public void periodic() {
    elevatorLeftController.setReference(armState.getExtension(), CANSparkMax.ControlType.kPosition);
    elevatorRightController.setReference(armState.getExtension(), CANSparkMax.ControlType.kPosition);
    elevatorWinchController.setReference(armState.getAngle(), CANSparkMax.ControlType.kPosition);
  }

  public Command getIntakeCommand() {
    return new StartEndCommand(() -> intakeFarMotor.set(INTAKE_SPEED), () -> intakeFarMotor.set(0), this);
  }
  public Command getOuttakeCommand() {
    return new StartEndCommand(() -> intakeFarMotor.set(-INTAKE_SPEED), () -> intakeFarMotor.set(0), this);
  }
}