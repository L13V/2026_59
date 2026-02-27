package org.ramtech.frc2026.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.FlywheelConstants;
import org.ramtech.frc2026.Constants.TurretConstants;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretIOReal implements TurretIO {
  // Motors
  private final TalonFX turretMotor =
      new TalonFX(TurretConstants.turretMotorId, Constants.CANivore); // Main Motor

  // Encoders
  private final CANcoder turretEncoderA =
      new CANcoder(TurretConstants.turretEncoderAId, Constants.CANivore);
  private final CANcoder turretEncoderB =
      new CANcoder(TurretConstants.turretEncoderBId, Constants.CANivore);

  // Configuration
  private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration encoderAConfig = new CANcoderConfiguration();
  private final CANcoderConfiguration encoderBConfig = new CANcoderConfiguration();

  private boolean turretMotorConfigured = false;
  public boolean turretEncoderAConfigured = false;
  public boolean turretEncoderBConfigured = false;

  // Status Signals

  // Control Methods
  private final VoltageOut voltageOut = new VoltageOut(0); // Control Method
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);


  public TurretIOReal() {
    // Complete the config
    turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    turretConfig.Slot0.kP = FlywheelConstants.kP_Slot0;
    turretConfig.Slot0.kI = FlywheelConstants.kI_Slot0;
    turretConfig.Slot0.kD = FlywheelConstants.kD_Slot0;
    turretConfig.Slot0.kS = FlywheelConstants.kS_Slot0;
    turretConfig.Slot0.kV = FlywheelConstants.kV_Slot0;
    turretConfig.Slot0.kA = FlywheelConstants.kA_Slot0;
    turretConfig.Slot0.kG = FlywheelConstants.kG_Slot0;

    encoderAConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderAConfig.MagnetSensor.MagnetOffset = 0.0;
    encoderAConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    encoderBConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderBConfig.MagnetSensor.MagnetOffset = 0.0;
    encoderBConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
  }

  public void configureDevices(TurretIOInputs inputs) {
    // Turret Motor
    if (!turretMotorConfigured && inputs.turretMotorConnected) { // Configure turret motor
      turretMotor.getConfigurator().apply(turretConfig);
      turretMotorConfigured = true;
    }
    inputs.turretMotorConfigured = turretMotorConfigured; // Update status on TurretIO
    // Encoder A
    if (!turretEncoderAConfigured && inputs.turretEncoderAConnected) { // Configure turret encoder A
      turretEncoderA.getConfigurator().apply(encoderAConfig);
      turretEncoderAConfigured = true;
    }
    inputs.turretEncoderAConfigured = turretEncoderAConfigured; // Update status on TurretIO
    // Encoder B
    if (!turretEncoderBConfigured && inputs.turretEncoderBConnected) { // Configure turret encoder B
      turretEncoderB.getConfigurator().apply(encoderBConfig);
      turretEncoderBConfigured = true;
    }
    inputs.turretEncoderBConfigured = turretEncoderBConfigured; // Update status on TurretIO
  }

  public void attemptCrt(TurretIOInputs inputs) {
    // If everything online and configured, calculate start angle and push to motor.
    if (turretMotorConfigured
        && inputs.turretMotorConnected
        && inputs.turretEncoderAConfigured
        && inputs.turretEncoderAConnected
        && inputs.turretEncoderBConfigured
        && inputs.turretEncoderBConnected) {
      // Prep CRT Solver
      var easyCrtConfig =
          new EasyCRTConfig(
              turretEncoderA.getPosition().asSupplier(), turretEncoderB.getPosition().asSupplier());
      // Solve and push to motor
      var easyCrtSolver = new EasyCRT(easyCrtConfig);
      easyCrtSolver
          .getAngleOptional()
          .ifPresent(
              mechAngle -> {
                turretMotor.setPosition(mechAngle);
              });
    }
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Set Motor Connection Status
    inputs.turretMotorConnected = turretMotor.isConnected();
    inputs.turretEncoderAConnected = turretEncoderA.isConnected();
    inputs.turretEncoderBConnected = turretEncoderB.isConnected();

    // Check and configure turret motor and encoders.
    configureDevices(inputs);
    // get motor outputs and push to TurretIO

    inputs.TurretMotorVoltage = turretMotor.getMotorVoltage().getValueAsDouble();
    inputs.TurretMotorPosition = turretMotor.getPosition().getValueAsDouble();
    inputs.turretMotorVelocity = turretMotor.getVelocity().getValueAsDouble();
    inputs.TurretMotorSupplyCurrent = turretMotor.getSupplyCurrent().getValueAsDouble();

    inputs.TurretEncoderAPosition = turretEncoderA.getPosition().getValueAsDouble();
    inputs.TurretEncoderBPosition = turretEncoderB.getPosition().getValueAsDouble();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case OFF:
        turretMotor.stopMotor();
        break;
      case VOLTAGE:
        turretMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint));
        break;
      case POSITION:
        turretMotor.setControl(motionMagicVoltage.withPosition(outputs.positionSetpoint));
    }
  }
}
