package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Arm extends SubsystemBase {
    TalonFX armMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID);
    SparkMax intakeMotor = new SparkMax(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    public Arm() {
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimit = 40;
        armConfig.CurrentLimits.SupplyCurrentLowerLimit = 80;
        armConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; //TODO find this value
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; //TODO find this value
        
        armConfig.Feedback.FeedbackRotorOffset = 0; // Sensor must be zeroed so arm is parallel to ground
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kG = 0;
        armConfig.Slot0.kV = 0;
        armConfig.Slot0.kA = 0;
        armConfig.Slot0.kP = 0;
        armConfig.Slot0.kI = 0;
        armConfig.Slot0.kD = 0;

        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armConfig.Feedback.FeedbackRemoteSensorID = 0;
    }
}
