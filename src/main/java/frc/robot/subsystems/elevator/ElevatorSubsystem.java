package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax motorL =
        new SparkMax(ElevatorConstants.LMotorCANId, MotorType.kBrushless);
    private final SparkMax motorR =
        new SparkMax(ElevatorConstants.RMotorCANId, MotorType.kBrushless);

    private final Encoder throughboreEncoder =
        new Encoder(ElevatorConstants.THROUGHBORE_CHANNEL_A, ElevatorConstants.THROUGHBORE_CHANNEL_B);

    public ElevatorSubsystem() {
        motorL.setInverted(ElevatorConstants.LMOTOR_INVERTED);
        motorR.setInverted(ElevatorConstants.RMOTOR_INVERTED);

        throughboreEncoder.reset();
        throughboreEncoder.setDistancePerPulse(1.0); // set scale in ticks
    }

    public void moveElevator(double speed) {
        double position = throughboreEncoder.get();
        double adjustedSpeed = applySoftLimits(speed, position);
        motorL.set(adjustedSpeed);
        motorR.set(adjustedSpeed);
    }

    public void stop() {
        motorL.stopMotor();
        motorR.stopMotor();
    }

// magic motion?!?!?!?!?!?!?
    private double applySoftLimits(double speed, double position) {
        double top = ElevatorConstants.MAX_HEIGHT_TICKS;
        double bottom = ElevatorConstants.MIN_HEIGHT_TICKS;
        double zone = ElevatorConstants.SLOW_ZONE;
        double min = ElevatorConstants.MIN_SPEED;

        if (speed > 0 && position >= top - zone) {
            double ratio = (top - position) / zone;
            return Math.max(min, speed * ratio);
        } else if (speed < 0 && position <= bottom + zone) {
            double ratio = (position - bottom) / zone;
            return Math.min(-min, speed * ratio);
        }
        return speed;
    }

    public double getPosition() {
        return throughboreEncoder.get();
    }

    public boolean atTop() {
        return throughboreEncoder.get() >= ElevatorConstants.MAX_HEIGHT_TICKS;
    }

    public boolean atBottom() {
        return throughboreEncoder.get() <= ElevatorConstants.MIN_HEIGHT_TICKS;
    }

    public void resetEncoder() {
        throughboreEncoder.reset();
    }
}
