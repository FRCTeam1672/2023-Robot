// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  // Whether tunable numbers can be changed from Shuffleboard.
  public static final boolean tuningMode = false;

  public static final class Elevator {
    public static final TunableNumber GROUND_ANGLE = new TunableNumber("Ground Angle", 6);
    public static final TunableNumber HYBRID_ANGLE = new TunableNumber("Hybrid Angle", 6);
    public static final TunableNumber MID_ANGLE = new TunableNumber("Mid Angle", -123.4);
    public static final TunableNumber HIGH_ANGLE = new TunableNumber("High Angle", -124);
    public static final TunableNumber SHELF_ANGLE = new TunableNumber("Shelf Angle", 6);
    public static final TunableNumber GROUND_EXTENSION = new TunableNumber("Ground Extension", 6);
    public static final TunableNumber HYBRID_EXTENSION = new TunableNumber("Hybrid Extension", 6);
    public static final TunableNumber MID_EXTENSION = new TunableNumber("Mid Extension", 50.5);
    public static final TunableNumber HIGH_EXTENSION = new TunableNumber("High Extension", 121);
    public static final TunableNumber SHELF_EXTENSION = new TunableNumber("Shelf Extension", 6);

    public static final TunableNumber INTAKE_CUBE_CAP = new TunableNumber("Intake Cube Cap", 74);
    public static final TunableNumber INTAKE_CONE_CAP = new TunableNumber("Intake Cone Cap", 60);

    public static final TunableNumber GAME_PIECE_RETRACT = new TunableNumber("Game Piece Retract", 20);
  }
  public static final class PIDConstants {
    public static final double Rkp = 0.12;
    public static final double Rki = 0.11764;
    public static final double Rkd = 0.112;
  }
}
