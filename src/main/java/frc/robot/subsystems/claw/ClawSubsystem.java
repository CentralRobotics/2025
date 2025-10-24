package frc.robot.subsystems.claw;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {

    private final SparkMax clawMotor = new SparkMax(ClawConstants.CLAW_MOTOR_ID, MotorType.kBrushless);


    
}
