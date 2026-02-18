package org.ramtech.frc2026.subsystems.shooter.tower;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  private static final DCMotor GEARBOX =
      new DCMotor(12, 7.09, 366, 2, Units.rotationsPerMinuteToRadiansPerSecond(6000), 1);

  // Hardware (Simulated)
  private final TalonFX towerMotor = new TalonFX(TowerConstants.towerMotorId);
  private final TalonFXSimState towerSimState = towerMotor.getSimState();

  // Physics
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, MOI_KG_M2, GEAR_RATIO), GEARBOX);

  // Controls
  private final VoltageOut voltageOut = new VoltageOut(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  public TowerIOSim() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 0.1; // TODO: Tune this for Sim/Real
    config.Slot0.kV = 0.12; // Approximate kV for Kraken
    towerMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(TowerIOInputs inputs) {
    // 1. Update Physics
    towerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(towerSimState.getMotorVoltage());
    sim.update(0.020); // 20ms loop time

    // 2. Update Sensors (Bridge Physics -> Talon)
    towerSimState.setRawRotorPosition(Units.radiansToRotations(sim.getAngularPositionRad()));
    towerSimState.setRotorVelocity(Units.radiansToRotations(sim.getAngularVelocityRadPerSec()));

    // 3. Update Inputs (Bridge Talon -> IO)
    inputs.towerConnected = true;
    inputs.towerConfigured = true;
    inputs.towerAppliedVoltage = towerMotor.getMotorVoltage().getValueAsDouble();
    inputs.towerVelocity = towerMotor.getVelocity().getValueAsDouble();
    inputs.towerSupplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void applyOutputs(TowerIOOutputs outputs) {
    switch (outputs.mode) {
      case OFF:
        towerMotor.stopMotor();
        break;
      case VOLTAGE:
        towerMotor.setControl(voltageOut.withOutput(outputs.voltage));
        break;
      case VELOCITY:
        towerMotor.setControl(velocityVoltage.withVelocity(outputs.velocity));
        break;
    }
  }
}
