package org.ramtech.frc2026.subsystems.intake;

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
import org.ramtech.frc2026.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
  // Physics constants
  private static final double GEAR_RATIO = 1.0; // TODO: Set actual gear ratio
  private static final double MOI_KG_M2 = 0.001; // TODO: Set actual MOI
  private static final DCMotor GEARBOX =
      new DCMotor(12, 7.09, 366, 2, Units.rotationsPerMinuteToRadiansPerSecond(6000), 1);

  // Hardware (Simulated)
  private final TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerMotorId);
  private final TalonFXSimState rollerSimState = rollerMotor.getSimState();

  // Physics
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, MOI_KG_M2, GEAR_RATIO), GEARBOX);

  // Controls
  private final VoltageOut rollerVoltageOut = new VoltageOut(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  public IntakeIOSim() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 0.1; // TODO: Tune this for Sim/Real
    config.Slot0.kV = 0.12; // Approximate kV for Kraken
    rollerMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // 1. Update Physics
    rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(rollerSimState.getMotorVoltage());
    sim.update(0.020); // 20ms loop time

    // 2. Update Sensors (Bridge Physics -> Talon)
    rollerSimState.setRawRotorPosition(Units.radiansToRotations(sim.getAngularPositionRad()));
    rollerSimState.setRotorVelocity(Units.radiansToRotations(sim.getAngularVelocityRadPerSec()));

    // 3. Update Inputs (Bridge Talon -> IO)
    inputs.rollerConnected = true;
    inputs.rollerConfigured = true;
    inputs.rollerVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerRps = rollerMotor.getVelocity().getValueAsDouble();
    inputs.rollerSupplyCurrent = sim.getCurrentDrawAmps();
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (outputs.mode) {
      case OFF:
        rollerMotor.stopMotor();
        break;
      case VOLTAGE:
        rollerMotor.setControl(rollerVoltageOut.withOutput(outputs.voltageSetpoint));
        break;
      case VELOCITY:
        rollerMotor.setControl(velocityVoltage.withVelocity(outputs.velocitySetpoint));
        break;
    }
  }
}
