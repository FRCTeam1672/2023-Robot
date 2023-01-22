package frc.robot;

import frc.robot.Constants.ArmState;

public class Node {
  public final Height height;
  public final Position position;

  public Node(Height height, Position position) {
    this.height = height;
    this.position = position;
  }

  public ArmState heightAsArmState() {
    switch(height) {
      case HIGH:
        return ArmState.HIGH;
      case MID:
        return ArmState.MID;
      case LOW:
        return ArmState.GROUND;
      default:
        return null;
    }
  }

  @Override
  public String toString() {
    return String.format("#%d %s", position.ordinal()+1, height.name());
  }

  public enum Height {
    HIGH,
    MID,
    LOW;
  }

  public enum Position {
    DS1_LEFT,
    DS1_MID,
    DS2_RIGHT,
    COOP_LEFT,
    COOP_MID,
    COOP_RIGHT,
    DS3_LEFT,
    DS3_MID,
    DS3_RIGHT;
  }
}
