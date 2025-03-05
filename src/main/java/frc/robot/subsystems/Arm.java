package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final TalonFXConfiguration armConfig = new TalonFXConfiguration();

    public Arm() {
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimit = 40;
        armConfig.CurrentLimits.SupplyCurrentLowerLimit = 80;
        armConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.FORWARD_LIMIT;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.REVERSE_LIMIT;
        
        armConfig.Feedback.FeedbackRotorOffset = 0; // Sensor must be zeroed so arm is parallel to ground
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kG = ArmConstants.kG;
        armConfig.Slot0.kV = ArmConstants.kV;
        armConfig.Slot0.kA = ArmConstants.kA;
        armConfig.Slot0.kP = ArmConstants.kP;
        armConfig.Slot0.kI = ArmConstants.kI;
        armConfig.Slot0.kD = ArmConstants.kD;

        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armConfig.Feedback.FeedbackRemoteSensorID = ArmConstants.ARM_CANCODER_ID;

        armMotor.getConfigurator().apply(armConfig);
    }

    public void setArmAngle(double angle) {
        double constrainAngle = MathUtil.clamp(angle, ArmConstants.REVERSE_LIMIT, ArmConstants.FORWARD_LIMIT);
        armMotor.set(constrainAngle);
    }

    public Command moveToAngle(double angle) {
        return run(() -> setArmAngle(angle));
    }

    public void stopArm() {
        armMotor.set(0.0);
    }

   public void moveIntake(boolean forward) {
        double speed = forward ? ArmConstants.INTAKE_SPEED : -ArmConstants.INTAKE_SPEED;
        intakeMotor.set(speed);
}

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    @Override
    public void periodic() {
    }
}
