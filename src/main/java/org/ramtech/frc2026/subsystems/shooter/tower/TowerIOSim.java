package org.ramtech.frc2026.subsystems.shooter.tower;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ramtech.frc2026.Constants.TowerConstants;

public class TowerIOSim implements TowerIO {
    // Physics constants
    private static final double GEAR_RATIO = 1.0; // TODO: Set actual gear ratio
    private static final double MOI_KG_M2 = 0.001; // TODO: Set actual MOI
    
    // Updated to represent a 2-motor gearbox
    private static final DCMotor GEARBOX = new DCMotor(12, 7.09, 366, 2,
            Units.rotationsPerMinuteToRadiansPerSecond(6000), 2);

    // Hardware (Simulated)
    private final TalonFX towerMotorA = new TalonFX(TowerConstants.towerMotorAId);
    private final TalonFX towerMotorB = new TalonFX(TowerConstants.towerMotorBId);
    
    private final TalonFXSimState towerSimStateA = towerMotorA.getSimState();
    private final TalonFXSimState towerSimStateB = towerMotorB.getSimState();

    // Physics
    private final DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, MOI_KG_M2, GEAR_RATIO),
            GEARBOX);

    // Controls
    private final VoltageOut voltageOut = new VoltageOut(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final StrictFollower follower = new StrictFollower(TowerConstants.towerMotorAId);

    public TowerIOSim() {
        var config = new TalonFXConfiguration();
        // Matched to the real TowerIOTalonFX config
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Slot0.kP = 0.1; // TODO: Tune this for Sim/Real
        config.Slot0.kV = 0.12; // Approximate kV for Kraken
        
        towerMotorA.getConfigurator().apply(config);
        towerMotorB.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(TowerIOInputs inputs) {
        // 1. Update Physics
        towerSimStateA.setSupplyVoltage(RobotController.getBatteryVoltage());
        towerSimStateB.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        // Since they are mechanically linked and B follows A, we can drive the sim with A's voltage
        sim.setInputVoltage(towerSimStateA.getMotorVoltage());
        sim.update(0.020); // 20ms loop time

        // 2. Update Sensors (Bridge Physics -> Talon)
        double positionRot = Units.radiansToRotations(sim.getAngularPositionRad());
        double velocityRps = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
        
        towerSimStateA.setRawRotorPosition(positionRot);
        towerSimStateA.setRotorVelocity(velocityRps);
        
        towerSimStateB.setRawRotorPosition(positionRot);
        towerSimStateB.setRotorVelocity(velocityRps);

        // 3. Update Inputs (Bridge Talon -> IO)
        inputs.signalsOk = StatusCode.OK;
        
        // Motor A
        inputs.towerMotorAConnected = true;
        inputs.towerMotorAConfigured = true;
        inputs.towerMotorAVoltage = towerMotorA.getMotorVoltage().getValueAsDouble();
        inputs.towerMotorAVelocity = towerMotorA.getVelocity().getValueAsDouble();
        inputs.towerMotorASupplyCurrent = sim.getCurrentDrawAmps() / 2.0; // Split current across 2 motors

        // Motor B
        inputs.towerMotorBConnected = true;
        inputs.towerMotorBConfigured = true;
        inputs.towerMotorBVoltage = towerMotorB.getMotorVoltage().getValueAsDouble();
        inputs.towerMotorBVelocity = towerMotorB.getVelocity().getValueAsDouble();
        inputs.towerMotorBSupplyCurrent = sim.getCurrentDrawAmps() / 2.0; // Split current across 2 motors
    }

    @Override
    public void applyOutputs(TowerIOOutputs outputs) {
        switch (outputs.mode) {
            case OFF:
                towerMotorA.stopMotor();
                towerMotorB.stopMotor();
                break;
            case VOLTAGE:
                towerMotorA.setControl(voltageOut.withOutput(outputs.voltageSetpoint));
                towerMotorB.setControl(follower);
                break;
            case VELOCITY:
                towerMotorA.setControl(velocityVoltage.withVelocity(outputs.velocitySetpoint));
                towerMotorB.setControl(follower);
                break;
        }
    }
}