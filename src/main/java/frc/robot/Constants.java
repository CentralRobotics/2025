// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(6);
  public static final int ControllerConstant = 2;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class ClimberConstants {
    public static final int MOTOR_ID = 31;
    public static final boolean MOTOR_INVERTED = false;

    public static final int SERVO_PWM = 0;
    public static final double RATCHET_ENGAGED_DEG = 30;
    public static final double RATCHET_DISENGAGED_DEG = 120;

    public static final int CLIMB_CURRENT_LIMIT_A = 40;
    public static final double CLIMB_UP_PCT = 0.90;
    public static final double LOWER_DOWN_PCT = -0.30; // let rope out
    public static final double MAX_OUTPUT = 1.0;

    // “Stall” detect
    public static final double STALL_CURRENT_A = 45.0;
    public static final double STALL_TIME_S = 0.20;
  }

  public static class ElevatorConstants {
    public static final int LMotorCANId = 15;
    public static final int RMotorCANId = 16;
    public static final boolean LMOTOR_INVERTED = false;
    public static final boolean RMOTOR_INVERTED = true;

    public static final Integer THROUGHBORE_CHANNEL_A = null;
    public static final Integer THROUGHBORE_CHANNEL_B = null;
    public static final double THROUGHBORE_DISTANCE_PER_PULSE = 1.0;

    // Throughbore configuration
    public static final double MAX_HEIGHT_TICKS = 5000;
    public static final double MIN_HEIGHT_TICKS = 0;
    public static final double SLOW_ZONE_TICKS = 250;
    public static final double SLOW_ZONE = 500; // ticks before top/bottom to start slowing

    // Motion config 
    public static final double MAX_SPEED = 1.0; // 1.0 may be too fast with two
    public static final double MIN_SPEED = 0.05; // crawl speed near limits

    // Internal relative encoder fallback
    public static final double ROTATIONS_TO_TOP = 85.0; // calibrate experimentally
    public static final double SLOW_ZONE_ROTATIONS = 5.0;
  }
}
