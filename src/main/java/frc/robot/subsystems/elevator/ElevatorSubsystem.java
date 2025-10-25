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
        motorL.set(speed);
        motorR.set(-speed);
    }

    public void stop() {
        double position = getPosition();
        System.out.println(position);
        motorL.stopMotor();
        motorR.stopMotor();
    }

   
    public double getPosition() {
        if (useThroughbore && throughboreEncoder != null) {
            return throughboreEncoder.get();
        } else {
            // Average both motors and apply gear ratio correction
            double avgMotorRotations = (internalEncoderL.getPosition() + internalEncoderR.getPosition()) / 2.0;
            return internalEncoderL.getPosition(); // converts to drum rotations
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