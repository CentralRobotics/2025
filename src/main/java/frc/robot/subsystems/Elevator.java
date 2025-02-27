package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    TalonFX elevatorMotor_1 = new TalonFX(Constants.MotorConstants.LEFT_ELEVATOR_MOTOR_ID);
    TalonFX elevatorMotor_2 = new TalonFX(Constants.MotorConstants.RIGHT_ELEVATOR_MOTOR_ID);
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    public Elevator() {
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(0);
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(0);

        elevatorMotor_1.getConfigurator().apply(elevatorConfig);
        elevatorMotor_2.getConfigurator().apply(elevatorConfig);
    }
    
}
