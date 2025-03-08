package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    TalonFX elevatorMotor_1 = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID);
    TalonFX elevatorMotor_2 = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID);
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    CANcoder sensor = new CANcoder(ElevatorConstants.ELEVATOR_CANCODER_ID);
    CANcoderConfiguration sensorConfig = new CANcoderConfiguration();

    private ElevatorSim m_elevatorSim = null;
    private DCMotor m_motorSim = DCMotor.getKrakenX60(2);
    private final TalonFXSimState leftSim = elevatorMotor_1.getSimState();
    private final TalonFXSimState rightSim = elevatorMotor_1.getSimState();
    private final CANcoderSimState sensorSim = sensor.getSimState();

    public Elevator() {

        // sensorConfig.MagnetSensor.MagnetOffset = 0;
        // sensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // sensor.getConfigurator().apply(sensorConfig);

        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 80;
        elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        elevatorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50; //TODO find this value
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; //TODO find this value
        
        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Slot0.kG = ElevatorConstants.kG;
        elevatorConfig.Slot0.kV = ElevatorConstants.kV;
        elevatorConfig.Slot0.kA = ElevatorConstants.kA;
        elevatorConfig.Slot0.kP = ElevatorConstants.kP;
        elevatorConfig.Slot0.kI = ElevatorConstants.kI;
        elevatorConfig.Slot0.kD = ElevatorConstants.kD;

        elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // elevatorConfig.Feedback.FeedbackRemoteSensorID = ElevatorConstants.ELEVATOR_CANCODER_ID;

        elevatorMotor_1.getConfigurator().apply(elevatorConfig);
        elevatorMotor_2.getConfigurator().apply(elevatorConfig);

        elevatorMotor_1.setPosition(0);
        elevatorMotor_2.setPosition(0);


        if (RobotBase.isSimulation()) {
            m_elevatorSim = new ElevatorSim(m_motorSim,
                    ElevatorConstants.kElevatorGearing,
                    ElevatorConstants.kElevatorCarriageMass,
                    ElevatorConstants.kElevatorDrumRadius,
                    ElevatorConstants.kElevatorMinHeightMeters,
                    ElevatorConstants.kElevatorMaxHeightMeters,
                    true,
                    Units.inchesToMeters(20),
                    0.02,
                    0.0);

        }
    }

    public void simulationPeriodic() {
        sensorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        //set input(voltage)
        m_elevatorSim.setInput(leftSim.getMotorVoltage(), rightSim.getMotorVoltage());

        //update-every 20 milliseconds
        m_elevatorSim.update(0.02);

        sensorSim.setRawPosition(convertDistanceToRotations(Meters.of(m_elevatorSim.getPositionMeters())));
        sensorSim.setVelocity(Elevator.convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond())).per(Second).in(RPM));

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    }

    public double getPositionMeters() {
        return convertRotationsToDistance(sensor.getPosition().getValueAsDouble());
    }

    public double getVelocityMetersPerSecond() {
        return convertRotationsToDistance(sensor.getVelocity().getValueAsDouble());
    }

    private double setHeight;
    public void reachGoal(double goal){
        System.out.println("setting elevator to " + goal);
        setHeight = goal;
        PositionDutyCycle command = new PositionDutyCycle(convertDistanceToRotations(Meters.of(goal)));
        elevatorMotor_1.setControl(command);
        elevatorMotor_2.setControl(command);
    }

    public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public Command setElevatorHeight(double height){
        return setGoal(height).until(()->aroundHeight(height));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

    public static double convertRotationsToDistance(Double rotation)
    {
        // m = e * (2*pi*r)/g
        return rotation * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
    }

    public static Angle convertDistanceToRotations(Distance distance)
    {
        // m/(2*pi*r)*g = e
        return Rotations.of((distance.in(Meters) / (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)));
    }

    public void set(double speed){
        VoltageOut command = new VoltageOut(-speed * 12);
        elevatorMotor_1.setControl(command);
        elevatorMotor_2.setControl(command);
    }

    public Command setElevator(double speed){
        return run(() -> set(speed));
    }

    public Command setElevator(CommandXboxController xbox){
        return run(() -> set(xbox.getLeftY()));
    }

     /**
     * Stop the control loop and motor output.
     */
    public void stop()
    {
        set(0);
    }

    /**
     * Update telemetry, including the mechanism visualization.
     */
    public void updateTelemetry()
    {
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Elevator1", elevatorMotor_1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator2", elevatorMotor_2.getPosition().getValueAsDouble());
    }
    
    //TODO: zero elevator
}
