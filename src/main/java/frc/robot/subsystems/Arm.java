package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    private final TalonFXConfiguration armConfig = new TalonFXConfiguration();
    CANcoder sensor = new CANcoder(ArmConstants.ARM_CANCODER_ID);
    CANcoderConfiguration sensorConfig = new CANcoderConfiguration();

    public Arm() {
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimit = 40;
        armConfig.CurrentLimits.SupplyCurrentLowerLimit = 80;
        armConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
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

        sensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        sensorConfig.MagnetSensor.MagnetOffset = -0.446;
        sensor.getConfigurator().apply(sensorConfig);

        double position = sensor.getAbsolutePosition().getValueAsDouble();
        if (position < ArmConstants.REVERSE_LIMIT)
            position += 1;
        sensor.setPosition(sensor.getAbsolutePosition().getValueAsDouble());
        System.out.println("setting position to " + position);
        // sensor.setPosition(ArmConstants.REVERSE_LIMIT);
    }

    public boolean aroundAngle(double angle){
        return aroundAngle(angle, ArmConstants.kArmDefaultTolerance);
    }
    public boolean aroundAngle(double angle, double tolerance){
        return MathUtil.isNear(angle,armMotor.getPosition().getValueAsDouble(),tolerance);
    }

    public void setArmAngle(double angle) {
        // double constrainAngle = MathUtil.clamp(angle, ArmConstants.REVERSE_LIMIT, ArmConstants.FORWARD_LIMIT);
        // armMotor.set(constrainAngle);
        PositionDutyCycle command = new PositionDutyCycle(angle);
        armMotor.setControl(command);
    }

    public Command moveToAngle(double angle) {
        return run(() -> setArmAngle(angle));
    }

    public void setArm(double speed){
        VoltageOut command = new VoltageOut(speed * 12);
        armMotor.setControl(command);
    }

    private double sensitivity (double in, double a) {
        //ax^3+(1-a)x
        return ((a*in*in*in)+(1-a)*in);
    }

    public Command manualArm(CommandXboxController xbox){
        return run(() -> {
            if (xbox.leftBumper().getAsBoolean() || Constants.alwaysManual) 
                setArm(sensitivity(xbox.getRightX(), 0.6));
                //cube input

        });
    }

    public void stopArm() {
        armMotor.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm", armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ArmSensor", sensor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ArmSensorAbs", sensor.getAbsolutePosition().getValueAsDouble());
    }
}
