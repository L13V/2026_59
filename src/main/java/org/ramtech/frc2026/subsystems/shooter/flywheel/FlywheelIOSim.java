package org.ramtech.frc2026.subsystems.shooter.flywheel;

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
import org.ramtech.frc2026.Constants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
  // Physics constants
  private static final double GEAR_RATIO = 1.0; // TODO: Set actual gear ratio
  private static final double MOI_KG_M2 = 0.004; // TODO: Set actual MOI
  private static final DCMotor GEARBOX =
      new DCMotor(12, 7.09, 366, 2, Units.rotationsPerMinuteToRadiansPerSecond(6000), 2);

  // Hardware (Simulated)
  private final TalonFX leftMotor = new TalonFX(FlywheelConstants.leftMotorId);
  private final TalonFX rightMotor = new TalonFX(FlywheelConstants.rightMotorId);
  private final TalonFXSimState leftSimState = leftMotor.getSimState();
  private final TalonFXSimState rightSimState = rightMotor.getSimState();

  // Physics
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, MOI_KG_M2, GEAR_RATIO), GEARBOX);

  // Controls
  private final VoltageOut voltageOut = new VoltageOut(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final StrictFollower follower = new StrictFollower(FlywheelConstants.leftMotorId);

  public FlywheelIOSim() {
    var leftConfig = new TalonFXConfiguration();
    var rightConfig = new TalonFXConfiguration();

    // Copy Configs from Constants/Real implementation
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftConfig.Slot0.kP = FlywheelConstants.kP_Slot0;
    leftConfig.Slot0.kI = FlywheelConstants.kI_Slot0;
    leftConfig.Slot0.kD = FlywheelConstants.kD_Slot0;
    leftConfig.Slot0.kS = FlywheelConstants.kS_Slot0;
    leftConfig.Slot0.kV = FlywheelConstants.kV_Slot0;
    leftConfig.Slot0.kA = FlywheelConstants.kA_Slot0;
    leftConfig.Slot0.kG = FlywheelConstants.kG_Slot0;

    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);

    // Set right motor to follow left in Sim logic
    rightMotor.setControl(follower);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // 1. Update Physics
    leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Drive physics with the leader's voltage
    sim.setInputVoltage(leftSimState.getMotorVoltage());
    sim.update(0.020);

    // 2. Update Sensors
    double pos = Units.radiansToRotations(sim.getAngularPositionRad());
    double vel = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    double current = sim.getCurrentDrawAmps() / 2.0; // Split current between motors

    leftSimState.setRawRotorPosition(pos);
    leftSimState.setRotorVelocity(vel);

    rightSimState.setRawRotorPosition(pos);
    rightSimState.setRotorVelocity(vel);

    // 3. Update Inputs
    inputs.leftSideConnected = true;
    inputs.rightSideConnected = true;
    inputs.leftSideConfigured = true;
    inputs.rightSideConfigured = true;

    inputs.leftSideMotorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightSideMotorVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftSideVelocity = leftMotor.getVelocity().getValueAsDouble();
    inputs.rightSideVelocity = rightMotor.getVelocity().getValueAsDouble();
    inputs.leftSideSupplyCurrent = current;
    inputs.rightSideSupplyCurrent = current;
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    // We only control the leader (Left), Right is set to follower in constructor
    if (outputs.mode == FlywheelIOOutputMode.VOLTAGE) {
      leftMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint));
    } else if (outputs.mode == FlywheelIOOutputMode.VELOCITY) {
      leftMotor.setControl(velocityVoltage.withVelocity(outputs.velocitySetpoint));
    } else {
      leftMotor.stopMotor();
    }
    rightMotor.setControl(follower);
  }
}
