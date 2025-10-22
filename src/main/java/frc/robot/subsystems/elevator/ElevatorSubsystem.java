package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkFlex motorL =
        new SparkFlex(ElevatorConstants.LMotorCANId, MotorType.kBrushless);
    private final SparkFlex motorR =
        new SparkFlex(ElevatorConstants.RMotorCANId, MotorType.kBrushless);

    private Encoder throughboreEncoder = null;
    private final RelativeEncoder internalEncoderL = motorL.getEncoder();
    private final RelativeEncoder internalEncoderR = motorR.getEncoder();
    

    private final boolean useThroughbore;

    public ElevatorSubsystem() {

        useThroughbore = ElevatorConstants.THROUGHBORE_CHANNEL_A != null
                      && ElevatorConstants.THROUGHBORE_CHANNEL_B != null;

        if (useThroughbore) {
            throughboreEncoder = new Encoder(
                ElevatorConstants.THROUGHBORE_CHANNEL_A,
                ElevatorConstants.THROUGHBORE_CHANNEL_B
            );
            throughboreEncoder.reset();
            throughboreEncoder.setDistancePerPulse(ElevatorConstants.THROUGHBORE_DISTANCE_PER_PULSE); //  ticks if scaled
            System.out.println("[Bionic|Elevator] Using Throughbore Encoder (ticks)");
        } else {
            internalEncoderL.setPosition(0);
            internalEncoderR.setPosition(0);
            System.out.println("[Bionic|Elevator] Using Internal REV Relative Encoders (rotations)");
        }
    }

    public void moveElevator(double speed) {
        double position = getPosition();
        double adjustedSpeed = applySoftLimits(speed, position);
        motorL.set(adjustedSpeed);
        motorR.set(adjustedSpeed);
    }

    public void stop() {
        motorL.stopMotor();
        motorR.stopMotor();
    }

    private double applySoftLimits(double speed, double position) {
        double top, bottom, zone;

        if (useThroughbore) {
            top = ElevatorConstants.MAX_HEIGHT_TICKS;
            bottom = ElevatorConstants.MIN_HEIGHT_TICKS;
            zone = ElevatorConstants.SLOW_ZONE_TICKS;
        } else {
            top = ElevatorConstants.ROTATIONS_TO_TOP;
            bottom = 0.0;
            zone = ElevatorConstants.SLOW_ZONE_ROTATIONS;
        }

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
        if (useThroughbore && throughboreEncoder != null) {
            return throughboreEncoder.get();
        } else {
            // Average both motors and apply gear ratio correction
            double avgMotorRotations = (internalEncoderL.getPosition() + internalEncoderR.getPosition()) / 2.0;
            return avgMotorRotations / ElevatorConstants.GEAR_RATIO; // converts to drum rotations
        }
    }

    public boolean atTop() {
        if (useThroughbore) {
            return getPosition() >= ElevatorConstants.MAX_HEIGHT_TICKS;
        } else {
            return getPosition() >= ElevatorConstants.ROTATIONS_TO_TOP;
        }
    }

    public boolean atBottom() {
        if (useThroughbore) {
            return getPosition() <= ElevatorConstants.MIN_HEIGHT_TICKS;
        } else {
            return getPosition() <= 0.0;
        }
    }

    public void resetEncoder() {
        if (useThroughbore && throughboreEncoder != null) {
            throughboreEncoder.reset();
        } else {
            internalEncoderL.setPosition(0);
            internalEncoderR.setPosition(0);
        }
    }
}
