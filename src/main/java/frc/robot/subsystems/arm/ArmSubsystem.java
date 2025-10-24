package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.RelativeEncoder;

public class ArmSubsystem extends SubsystemBase{
      private final SparkMax armMotor =
         new SparkMax(ArmConstants.ARM_ROLLER_MOTORID_A, MotorType.kBrushless);

        private final RelativeEncoder armEncoder;

    //constructor below
    public ArmSubsystem(){
        armEncoder = armMotor.getEncoder();
    }

    public void setMotorPower(double power){
        armMotor.set(power);
    }

    public void stopMotor(){
        armMotor.stopMotor();
    }
    
}
