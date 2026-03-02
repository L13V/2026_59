package org.ramtech.frc2026.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import org.ramtech.frc2026.Constants.HoodConstants;

public class HoodIOSim implements HoodIO {
	// Physics constants
	private static final double GEAR_RATIO = 10.0; // Estimate
	private static final double MOI_KG_M2 = 0.005; // Estimate
	private static final DCMotor GEARBOX = DCMotor.getFalcon500(1);

	// Hardware (Simulated)
	private final TalonFX hoodMotor = new TalonFX(HoodConstants.hoodMotorId);
	private final TalonFXSimState hoodSimState = hoodMotor.getSimState();

	// Physics
	private final DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, MOI_KG_M2, GEAR_RATIO),
			GEARBOX);

	// Controls
	private final VoltageOut voltageOut = new VoltageOut(0);
	private final PositionVoltage positionVoltage = new PositionVoltage(0);

	public HoodIOSim() {
		var hoodConfig = new TalonFXConfiguration();

		// Copy Configs from Constants/Real implementation
		hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		hoodConfig.Slot0.kP = HoodConstants.kP_Slot0;
		hoodConfig.Slot0.kI = HoodConstants.kI_Slot0;
		hoodConfig.Slot0.kD = HoodConstants.kD_Slot0;
		hoodConfig.Slot0.kS = HoodConstants.kS_Slot0;
		hoodConfig.Slot0.kV = HoodConstants.kV_Slot0;
		hoodConfig.Slot0.kA = HoodConstants.kA_Slot0;
		hoodConfig.Slot0.kG = HoodConstants.kG_Slot0;

		hoodMotor.getConfigurator().apply(hoodConfig);
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {
		// 1. Update Physics
		hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

		sim.setInputVoltage(hoodSimState.getMotorVoltage());
		sim.update(0.020);

		// 2. Update Sensors
		double pos = Units.radiansToRotations(sim.getAngularPositionRad());
		double vel = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());

		hoodSimState.setRawRotorPosition(pos);
		hoodSimState.setRotorVelocity(vel);

		// 3. Update Inputs
		inputs.hoodConnected = true;
		inputs.hoodConfigured = true;

		inputs.hoodMotorVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();
		inputs.hoodSupplyCurrent = sim.getCurrentDrawAmps();
		inputs.hoodPosition = hoodMotor.getPosition().getValueAsDouble();
		inputs.hoodVelocity = hoodMotor.getVelocity().getValueAsDouble();
	}

	@Override
	public void applyOutputs(HoodIOOutputs outputs) {
		if (outputs.mode == HoodIOOutputMode.VOLTAGE) {
			hoodMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint));
		} else if (outputs.mode == HoodIOOutputMode.POSITION) {
			hoodMotor.setControl(positionVoltage.withPosition(outputs.positionSetpoint));
		} else {
			hoodMotor.stopMotor();
		}
	}
}
