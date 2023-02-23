package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.elevator.RunForCommand;

public class ArmSubsystem extends SubsystemBase {
    private boolean calibrating = false;

    private final CANSparkMax rIntake = new CANSparkMax(21, MotorType.kBrushless);
    private final CANSparkMax lIntake = new CANSparkMax(22, MotorType.kBrushless);

    private final CANSparkMax rElevator = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax lElevator = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax winch = new CANSparkMax(13, MotorType.kBrushless);
    
    private final DigitalInput bottomElevatorLimitSwitch = new DigitalInput(9);

    public ArmSubsystem() {
        rIntake.follow(lIntake);
        lElevator.follow(rElevator, true);

        SmartDashboard.putData("Home", Commands.runOnce(() -> {
            if (calibrating) winch.getEncoder().setPosition(0);
            calibrating = !calibrating;
        }, this).ignoringDisable(true));
    }

    /** Increase elevator angle. */
    public void moveUp() {
        if (winch.getEncoder().getPosition() >= -3 && !calibrating) {
            stopWinch();
        } else {
            winch.set(0.75);
        }
    }
    /** Decrease elevator angle. */
    public void moveDown() {
        winch.set(-0.5);
    }

    /** Increase elevator extension. */
    public void extend() {
        // extend it a couple of inches 😏
        rElevator.set(0.2);
    }
    /** Decrease elevator extension. */
    public void retract() {
        // if limit switch trigger, *do not move*
        if (!bottomElevatorLimitSwitch.get()) {
            lElevator.getEncoder().setPosition(0);
            rElevator.getEncoder().setPosition(0);
            stopElevator();
        } else {
            // sheathe it, you heathen!
            rElevator.set(-0.2);
        }
    }

    public void intake() {
        lIntake.set(-0.7);
    }
    public void outtake() {
        lIntake.set(1);
    }

    public void stopWinch() {
        winch.stopMotor();
    }
    public void stopElevator() {
        lElevator.stopMotor();
        rElevator.stopMotor();
    }
    public void stopIntake() {
        lIntake.stopMotor();
        rIntake.stopMotor();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LElevator Encoder: ", lElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("RElevator Encoder: ", rElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Winch Elevator Encoder: ", winch.getEncoder().getPosition());
        SmartDashboard.putBoolean("Calibrating", calibrating);
    }

    public Command setAngle(double encoderPosition) {
        double ERROR = 0.1;

        return Commands.run(() -> {
            double errorDirection = Math.signum(encoderPosition - winch.getEncoder().getPosition());
            double errorProximity = encoderPosition - winch.getEncoder().getPosition() < 2*ERROR ? 0.5 : 1;
            double elevatorSpeed =  errorDirection * errorProximity * 0.55;

            winch.set(elevatorSpeed);
        }).until(() -> Math.abs(lElevator.getEncoder().getPosition() - encoderPosition) < ERROR);
    }
    public Command setExtension(double encoderPosition) {
        double ERROR = 0.1;

        return Commands.run(() -> {
            double errorDirection = Math.signum(encoderPosition - lElevator.getEncoder().getPosition());
            double errorProximity = encoderPosition - lElevator.getEncoder().getPosition() < 2*ERROR ? 0.5 : 1;
            double elevatorSpeed =  errorDirection * errorProximity * 0.2;

            lElevator.set(elevatorSpeed);
        }).until(() -> Math.abs(lElevator.getEncoder().getPosition() - encoderPosition) < ERROR);
    }

    public Command getSubstationIntakeCommand() {
        return Commands
                .parallel(
                        Commands.run(this::moveUp).until(() -> {
                            return winch.getEncoder().getPosition() >= 500 ? true : false;
                        }),
                        Commands.run(this::extend).until(() -> {
                            return lElevator.getEncoder().getPosition() >= 500 ? true : false;
                        }))
                .andThen(new RunForCommand(this::intake, 3), null)
                .andThen(
                        Commands.parallel(
                                Commands.run(this::moveDown).until(() -> {
                                    return winch.getEncoder().getPosition() <= 0 ? true : false;
                                }),
                                Commands.run(this::retract).until(() -> {
                                    return lElevator.getEncoder().getPosition() <= 0 ? true : false;
                                })));
    }

    public Command getScoreHighCommand() {
        return Commands.parallel(
            setAngle(60),
            setExtension(120)
        ).andThen(new RunForCommand(this::outtake, 3));
    }
}
