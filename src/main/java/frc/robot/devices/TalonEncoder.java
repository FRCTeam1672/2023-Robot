package frc.robot.devices;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonEncoder {
  private final WPI_TalonSRX motor;
  private double scaleFactor = 1;

  public TalonEncoder(WPI_TalonSRX motor) {
    this.motor = motor;
  }
  public TalonEncoder(WPI_TalonSRX motor, double scaleFactor) {
    this.motor = motor;
    this.scaleFactor = scaleFactor;
  }

  public void resetPosition() {
    motor.setSelectedSensorPosition(0);
  }

  public double getDistance() {
    return motor.getSelectedSensorPosition() / 4096.0 * scaleFactor;
  }
  public double getVelocity() {
    return motor.getSelectedSensorVelocity() * 10.0 / 4096.0 * scaleFactor;
  }
}