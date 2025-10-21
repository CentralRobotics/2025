package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax motorL = new SparkMax(ElevatorConstants.LMotorCANId, MotorType.kBrushless);
    SparkMax motorR = new SparkMax(ElevatorConstants.RMotorCANId, MotorType.kBrushless);

    private final RelativeEncoder kElevatorRelativeEncoderLeft = motorL.getEncoder();
    private final RelativeEncoder kElevatorRelativeEncoderRight = motorR.getEncoder();



    //stall detection
    private final Timer stallTimer = new Timer();

    public ElevatorSubsystem(){
        motorL.setInverted(ElevatorConstants.LMOTOR_INVERTED);
        motorR.setInverted(ElevatorConstants.RMOTOR_INVERTED);

        kElevatorRelativeEncoderLeft.setPosition(0.0);
        kElevatorRelativeEncoderRight.setPosition(0.0);

        stallTimer.stop();
        stallTimer.reset();

    }
    
}
