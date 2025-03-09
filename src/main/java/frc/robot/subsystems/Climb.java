package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    PneumaticHub m_pH = new PneumaticHub(1);
    Solenoid m_solenoidClimb = m_pH.makeSolenoid(0);

    public Climb() {
        m_pH.enableCompressorAnalog(100,120);
        retractClimb();
    }

    public void extendClimb() {
        System.out.println("extending climb");
        m_solenoidClimb.set(true);
    }
    
    public void retractClimb() {
        System.out.println("retracting climb");
        m_solenoidClimb.set(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pressure", m_pH.getPressure(0));
    }
}
