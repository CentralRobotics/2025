package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();


    public Intake() {
        config.smartCurrentLimit(80).idleMode(IdleMode.kBrake);
        intakeMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.clearFaults();
    }

    public void set(double speed) {
        // double speed = forward ? ArmConstants.INTAKE_SPEED : -ArmConstants.INTAKE_SPEED;
        intakeMotor.set(speed);
    }

    public Command manualIntake(CommandXboxController xbox){
        return run(() -> {
            // if (xbox.leftBumper().getAsBoolean())
            //     set(1);
            // else if (xbox.rightBumper().getAsBoolean())
            //     set(-1);
            // else
            //     set(0);
            double speed = xbox.getLeftTriggerAxis() - xbox.getRightTriggerAxis();
            set(speed);
        });
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public double getCurrentDrawAmps(){
        return intakeMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
    }
}
