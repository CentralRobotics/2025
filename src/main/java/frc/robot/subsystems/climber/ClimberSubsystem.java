package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {

 

  SparkMax motor = new SparkMax(1, MotorType.kBrushless);
  

 private final RelativeEncoder kClimbRelativeEncoder = motor.getEncoder();
  private final Servo ratchet = new Servo(ClimberConstants.SERVO_PWM);

  private boolean ratchetEngaged = true;
  private double targetPercent = 0.0;

  // Stall detection
  private final Timer stallTimer = new Timer();

  public ClimberSubsystem() {
    motor.setInverted(ClimberConstants.MOTOR_INVERTED);
    kClimbRelativeEncoder.setPosition(0.0);

    engageRatchet(); // start safe
    stallTimer.stop();
    stallTimer.reset();
  }

  // ======= SERVO / RATCHET =======
  public void engageRatchet() {
    ratchet.setAngle(ClimberConstants.RATCHET_ENGAGED_DEG);
    ratchetEngaged = true;
  }

  public void disengageRatchet() {
    ratchet.setAngle(ClimberConstants.RATCHET_DISENGAGED_DEG);
    ratchetEngaged = false;
  }

  public void toggleRatchet() {
    if (ratchetEngaged) disengageRatchet(); else engageRatchet();
  }

  public boolean isRatchetEngaged() { return ratchetEngaged; }

  // ======= CONTROL =======
  /** Direct percent control with interlocks. Positive = climb up (pull in line). */
  public void setPercent(double pct) {
    // Safety: don’t backdrive against an engaged ratchet
    if (pct < 0 && ratchetEngaged) {
      pct = 0; // block lowering if pawl is engaged
    }
    targetPercent = MathUtil.clamp(pct, -ClimberConstants.MAX_OUTPUT, ClimberConstants.MAX_OUTPUT);
    motor.set(targetPercent);
  }

  /** Convenience: climb up at preset speed (ratchet may stay engaged). */
  public void climbUp() {
    // It’s OK to keep the ratchet engaged while going up—many designs ratchet-click as they climb.
    setPercent(ClimberConstants.CLIMB_UP_PCT);
  }

  /** Convenience: lower down (requires the ratchet to be DISENGAGED). */
  public void lowerDown() {
    if (ratchetEngaged) disengageRatchet();
    setPercent(ClimberConstants.LOWER_DOWN_PCT);
  }

  public void stop() {
    targetPercent = 0.0;
    motor.set(0.0);
  }

  public double getCurrent() { return motor.getOutputCurrent(); }
  public double getRotations() { return kClimbRelativeEncoder.getPosition(); }

  public boolean isStalled() {
    double amps = getCurrent();
    if (amps >= ClimberConstants.STALL_CURRENT_A) {
      if (!stallTimer.isRunning()) stallTimer.restart();
    } else {
      stallTimer.stop();
      stallTimer.reset();
    }
    return stallTimer.hasElapsed(ClimberConstants.STALL_TIME_S);
  }

  @Override
  public void periodic() {

  }
}
