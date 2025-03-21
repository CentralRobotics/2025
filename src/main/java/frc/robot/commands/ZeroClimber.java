package frc.robot.commands;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ZeroClimber extends Command {
    private final Climb climb;

    public ZeroClimber(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize()
    {
        SoftwareLimitSwitchConfigs disable = new SoftwareLimitSwitchConfigs();
        disable.ForwardSoftLimitEnable = false;
        disable.ReverseSoftLimitEnable = false;
        climb.getMotor().getConfigurator().apply(disable);
        climb.getMotor().setControl(new VoltageOut(1));
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return climb.getMotor().getStatorCurrent().getValueAsDouble() > 3;
    }

    @Override
    public void end(boolean interrupted)
    {
        SoftwareLimitSwitchConfigs enable = new SoftwareLimitSwitchConfigs();
        enable.ForwardSoftLimitEnable = true;
        enable.ReverseSoftLimitEnable = true;
        climb.getMotor().getConfigurator().apply(enable);
    }
    
}
