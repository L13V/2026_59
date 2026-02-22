package org.ramtech.frc2026.subsystems.shooter.turret;

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
import org.ramtech.frc2026.Constants.FlywheelConstants;
import org.ramtech.frc2026.Constants.HoodConstants;

public class TurretIOSim implements TurretIO {
  // Physics constants
  private static final double GEAR_RATIO = 10.0; // Estimate
  private static final double MOI_KG_M2 = 0.005; // Estimate
  private static final DCMotor GEARBOX = DCMotor.getFalcon500(1);

  // Hardware (Simulated)
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.hoodMotorId);
  private final TalonFXSimState hoodSimState = hoodMotor.getSimState();

  // Physics
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, MOI_KG_M2, GEAR_RATIO), GEARBOX);

  // Controls
  private final VoltageOut voltageOut = new VoltageOut(0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0);

  public TurretIOSim() {
    var hoodConfig = new TalonFXConfiguration();

    // Copy Configs from Constants/Real implementation
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.Slot0.kP = FlywheelConstants.kP_Slot0;
    hoodConfig.Slot0.kI = FlywheelConstants.kI_Slot0;
    hoodConfig.Slot0.kD = FlywheelConstants.kD_Slot0;
    hoodConfig.Slot0.kS = FlywheelConstants.kS_Slot0;
    hoodConfig.Slot0.kV = FlywheelConstants.kV_Slot0;
    hoodConfig.Slot0.kA = FlywheelConstants.kA_Slot0;
    hoodConfig.Slot0.kG = FlywheelConstants.kG_Slot0;

    hoodMotor.getConfigurator().apply(hoodConfig);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
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
    inputs.turretMotorConnected = true;
    inputs.turretMotorConfigured = true;

    inputs.TurretMotorVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.TurretMotorSupplyCurrent = sim.getCurrentDrawAmps();
    inputs.TurretMotorPosition = hoodMotor.getPosition().getValueAsDouble();
    inputs.turretMotorVelocity = hoodMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    if (outputs.mode == TurretIOOutputMode.VOLTAGE) {
      hoodMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint));
    } else if (outputs.mode == TurretIOOutputMode.POSITION) {
      hoodMotor.setControl(positionVoltage.withPosition(outputs.positionSetpoint));
    } else {
      hoodMotor.stopMotor();
    }
  }
}
