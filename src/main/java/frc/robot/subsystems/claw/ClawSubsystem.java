package frc.robot.subsystems.claw;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {

    private final SparkMax clawMotor = new SparkMax(ClawConstants.CLAW_MOTOR_ID, MotorType.kBrushless);

    private final Timer stallTimer = new Timer();

    public ClawSubsystem() {
        SparkBaseConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);
        clawMotor.configure(config, null, null);
    }

    /** Run claw rollers at a given percent (-1 to 1) */
    public void setPower(double power) {
        clawMotor.set(power);
    }

    /** Stop the claw motor */
    public void stop() {
        clawMotor.stopMotor();
    }

    public double getCurrentInAmps() {
        return clawMotor.getOutputCurrent();
    }

    /** Detects if the claw motor is stalled (over threshold for time) */
    public boolean isStalled() {
        double current = getCurrentInAmps();
        if (current > ClawConstants.STALL_GATE_THRESHOLD) {
            if (!stallTimer.isRunning()) stallTimer.restart();
            return stallTimer.hasElapsed(ClawConstants.STALL_TIME_SECONDS);
        } else {
            stallTimer.stop();
            stallTimer.reset();
            return false;
        }
    }
    
}
