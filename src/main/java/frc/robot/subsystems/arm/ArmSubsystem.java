package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ArmSubsystem {
     private final SparkMax twinRollerAMotor =
        new SparkMax(ArmConstants.ARM_ROLLER_MOTORID_A, MotorType.kBrushless);
    private final SparkMax twinRollerBMotor =
        new SparkMax(ArmConstants.ARM_ROLLER_MOTORID_B, MotorType.kBrushless);
    private final SparkMax armArticulationMotor = new SparkMax(ArmConstants.ARM_ARTICULATE_MOTORID, MotorType.kBrushless);

}
