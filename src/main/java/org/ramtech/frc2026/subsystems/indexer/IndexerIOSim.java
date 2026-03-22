package org.ramtech.frc2026.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import org.ramtech.frc2026.Constants.IndexerConstants;

public class IndexerIOSim implements IndexerIO {
	// Physics constants
	private static final double GEAR_RATIO = 1.0;
	private static final double MOI_KG_M2 = 0.001;
	private static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);

	// Hardware (Simulated)
	private final TalonFX spindexerMotor = new TalonFX(IndexerConstants.indexerMotorID);
	private final TalonFX starMotor = new TalonFX(IndexerConstants.indexerMotorID);
	private final TalonFXSimState spindexerSimState = spindexerMotor.getSimState();
	private final TalonFXSimState starSimState = starMotor.getSimState();

	// Physics
	private final DCMotorSim spindexerSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(GEARBOX, MOI_KG_M2, GEAR_RATIO), GEARBOX);
	private final DCMotorSim starSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(GEARBOX, MOI_KG_M2, GEAR_RATIO), GEARBOX);

	// Controls
	private final VoltageOut voltageOut = new VoltageOut(0);

	public IndexerIOSim() {
		var config = new TalonFXConfiguration();
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		spindexerMotor.getConfigurator().apply(config);
		starMotor.getConfigurator().apply(config);
	}

	@Override
	public void updateInputs(IndexerIOInputs inputs) {
		// 1. Update Physics
		spindexerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
		starSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

		spindexerSim.setInputVoltage(spindexerSimState.getMotorVoltage());
		starSim.setInputVoltage(starSimState.getMotorVoltage());

		spindexerSim.update(0.020);
		starSim.update(0.020);

		// 2. Update Sensors
		spindexerSimState.setRawRotorPosition(Units.radiansToRotations(spindexerSim.getAngularPositionRad()));
		spindexerSimState.setRotorVelocity(Units.radiansToRotations(spindexerSim.getAngularVelocityRadPerSec()));

		starSimState.setRawRotorPosition(Units.radiansToRotations(starSim.getAngularPositionRad()));
		starSimState.setRotorVelocity(Units.radiansToRotations(starSim.getAngularVelocityRadPerSec()));

		// 3. Update Inputs
		inputs.indexerConnected = true;
	}

	@Override
	public void applyOutputs(IndexerIOOutputs outputs) {
		if (outputs.mode == IndexerIOOutputMode.VOLTAGE) {
			spindexerMotor.setControl(voltageOut.withOutput(outputs.indexerVoltageSetpoint));
		} else {
			spindexerMotor.stopMotor();
		}
	}
}
