package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Node;
import frc.robot.Targeter;
import frc.robot.Node.Translation;
import frc.robot.commands.elevator.TimerCommand;

import static frc.robot.Constants.Elevator.*;

public class ArmSubsystem extends SubsystemBase {
    private boolean calibrating = Constants.tuningMode;

    private final CANSparkMax rIntake = new CANSparkMax(21, MotorType.kBrushless);
    private final CANSparkMax lIntake = new CANSparkMax(22, MotorType.kBrushless);

    private final CANSparkMax rElevator = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax lElevator = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax winch = new CANSparkMax(13, MotorType.kBrushless);
    
    private final DigitalInput bottomElevatorLimitSwitch = new DigitalInput(9);
    private final DigitalInput winchAngleLimitSwitch = new DigitalInput(8);

    private final Targeter targeter;

    public ArmSubsystem(Targeter targeter) {
        this.targeter = targeter;
        rIntake.follow(lIntake);
        lElevator.follow(rElevator, true);

        SmartDashboard.putData("Home", Commands.runOnce(() -> {
            if (calibrating) winch.getEncoder().setPosition(0);
            calibrating = !calibrating;
        }, this).ignoringDisable(true));
    }

    /** Increase elevator angle. */
    public void moveUp() {
        if (!winchAngleLimitSwitch.get()) {
            stopWinch();
        } else {
            winch.set(1);
        }
    }
    /** Decrease elevator angle. */
    public void moveDown() {
        if(winch.getEncoder().getPosition() <= -222){
            stopWinch();
            return;
        } 
        winch.set(-1);
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
        double speed = targeter.getTargetNode().getTranslation() == Translation.CENTER ? 0.5 : 1.0;
        lIntake.set(speed);
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

    private boolean isStowed() {
        return !(bottomElevatorLimitSwitch.get() || winchAngleLimitSwitch.get());
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LElevator Encoder: ", lElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("RElevator Encoder: ", rElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Winch Elevator Encoder: ", winch.getEncoder().getPosition());
        SmartDashboard.putBoolean("Calibrating", calibrating);

        SmartDashboard.putNumber("Intake Current", this.lIntake.getOutputCurrent()+this.rIntake.getOutputCurrent());

        if(!bottomElevatorLimitSwitch.get()) {
            lElevator.getEncoder().setPosition(0);
            rElevator.getEncoder().setPosition(0);
        }
        if (!winchAngleLimitSwitch.get()) {
            winch.getEncoder().setPosition(0);
        }
    }

    public Command setAngle(double encoderPosition) {
        double ERROR = 4;

        return Commands.run(() -> {
            double errorDirection = Math.signum(encoderPosition - winch.getEncoder().getPosition());
            double errorProximity = encoderPosition - winch.getEncoder().getPosition() < 2*ERROR ? 0.5 : 1;
            double winchSpeed =  errorDirection * errorProximity * 0.8;

            winch.set(MathUtil.clamp(winchSpeed, -0.85, 0.85));
        }).until(() -> Math.abs(winch.getEncoder().getPosition() - encoderPosition) < ERROR)
        .andThen(this::stopWinch);
    }
    public Command setExtension(double encoderPosition) {
        double ERROR = 2;

        return Commands.run(() -> {
            double errorDirection = Math.signum(encoderPosition - rElevator.getEncoder().getPosition());
            double errorProximity = encoderPosition - rElevator.getEncoder().getPosition() < 2*ERROR ? 0.5 : 1;
            double elevatorSpeed =  errorDirection * 0.3;

            rElevator.set(MathUtil.clamp(elevatorSpeed, -0.3, 0.3));
        }).until(() -> Math.abs(rElevator.getEncoder().getPosition() - encoderPosition) < ERROR)
        .andThen(this::stopElevator);
    }

    private Command getStowCommand() {
        return Commands.parallel(
            Commands.run(this::retract), Commands.run(this::moveUp), Commands.runOnce(this::stopIntake)
        ).until(this::isStowed);
    }

    public Command getIntakeCommand() {
        double currentCap = targeter.getTargetNode().getTranslation() == Translation.CENTER ? INTAKE_CUBE_CAP.get() : INTAKE_CONE_CAP.get();

        return Commands.run(this::intake)
            .until(() -> this.lIntake.getOutputCurrent()+this.rIntake.getOutputCurrent() > currentCap)
            .andThen(() -> this.lIntake.set(-0.2));
    }

    private Command getScoreHybridCommand() {
        return Commands.parallel(
            setAngle(HYBRID_ANGLE.get()),
            setExtension(HYBRID_EXTENSION.get())
        ).andThen(new TimerCommand(this::outtake, 1))
        .finallyDo(e -> getStowCommand().schedule());
    }

    private Command getScoreMidCommand() {
        return Commands.parallel(
            setAngle(MID_ANGLE.get()),
            setExtension(MID_EXTENSION.get())
        ).andThen(new TimerCommand(this::outtake,1))
        .finallyDo(e -> getStowCommand().schedule());
    }

    private Command getScoreHighCommand() {
        return Commands.parallel(
            setAngle(HIGH_ANGLE.get()),
            setExtension(HIGH_EXTENSION.get())
        ).andThen(new TimerCommand(this::outtake, 1))
        .finallyDo(e -> getStowCommand().schedule());
    }

    public Command getScoreCommand(Node.Height nodeHeight) {
        switch(nodeHeight) {
            case HIGH: return getScoreHighCommand();
            case MID: return getScoreMidCommand();
            case HYBRID: return getScoreHybridCommand();
            default: return null;
        }
    }

    public Command getShelfIntakeCommand() {
        return Commands.parallel(
            setAngle(SHELF_ANGLE.get()),
            setExtension(SHELF_EXTENSION.get()),
            getIntakeCommand()
        )
        .finallyDo(e -> getStowCommand().schedule());
    }

    public Command getGroundIntakeCommand() {
        return Commands.parallel(
            setAngle(GROUND_ANGLE.get()),
            setExtension(GROUND_EXTENSION.get()),
            getIntakeCommand()
        )
        .finallyDo(e -> getStowCommand().schedule());
    }
}
