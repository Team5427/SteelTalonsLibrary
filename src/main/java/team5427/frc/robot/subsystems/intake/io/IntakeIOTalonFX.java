package team5427.frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.intake.IntakeConstants;
import team5427.lib.motors.SteelTalonFX;

public class IntakeIOTalonFX implements IntakeIO {
  private SteelTalonFX rollerMotor;
  private SteelTalonFX pivotMotor;

  private StatusSignal<Angle> pivotMotorPosition;
  private StatusSignal<AngularVelocity> pivotMotorAngularVelocity;
  private StatusSignal<AngularAcceleration> pivotMotorAngularAcceleration;

  private StatusSignal<AngularVelocity> rollerMotorAngularVelocity;
  private StatusSignal<AngularAcceleration> rollerMotorAngularAcceleration;

  private StatusSignal<Current> rollerMotorCurrent;
  private StatusSignal<Current> pivotMotorCurrent;

  private StatusSignal<Voltage> rollerMotorVoltage;
  private StatusSignal<Voltage> pivotMotorVoltage;

  private StatusSignal<Temperature> rollerMotorTemperature;
  private StatusSignal<Temperature> pivotMotorTemperature;

  private boolean isRollerMotorDisabled = false;
  private boolean isPivotMotorDisabled = false;

  public IntakeIOTalonFX() {
    rollerMotor = new SteelTalonFX(IntakeConstants.kRollerMotorCanId);
    pivotMotor = new SteelTalonFX(IntakeConstants.kPivotMotorCanId);

    rollerMotor.apply(IntakeConstants.kRollerMotorConfiguration);
    pivotMotor.apply(IntakeConstants.kPivotMotorConfiguration);

    pivotMotor.setEncoderPosition(IntakeConstants.kPivotStartingRotation);
    rollerMotor.setEncoderPosition(0.0);

    pivotMotorPosition = pivotMotor.getTalonFX().getPosition();
    pivotMotorAngularVelocity = pivotMotor.getTalonFX().getVelocity();
    pivotMotorAngularAcceleration = pivotMotor.getTalonFX().getAcceleration();

    rollerMotorAngularVelocity = rollerMotor.getTalonFX().getVelocity();
    rollerMotorAngularAcceleration = rollerMotor.getTalonFX().getAcceleration();

    rollerMotorCurrent = rollerMotor.getTalonFX().getStatorCurrent();
    pivotMotorCurrent = pivotMotor.getTalonFX().getStatorCurrent();

    rollerMotorVoltage = rollerMotor.getTalonFX().getMotorVoltage();
    pivotMotorVoltage = pivotMotor.getTalonFX().getMotorVoltage();

    rollerMotorTemperature = rollerMotor.getTalonFX().getDeviceTemp();
    pivotMotorTemperature = pivotMotor.getTalonFX().getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kHighPriorityUpdateFrequency, pivotMotorPosition, rollerMotorAngularVelocity);

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kMediumPriorityUpdateFrequency,
        pivotMotorAngularAcceleration,
        pivotMotorAngularVelocity,
        rollerMotorAngularAcceleration,
        rollerMotorCurrent,
        pivotMotorCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kLowPriorityUpdateFrequency,
        pivotMotorTemperature,
        pivotMotorVoltage,
        rollerMotorTemperature,
        rollerMotorVoltage);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerMotorConnected = rollerMotor.getTalonFX().isConnected();
    inputs.pivotMotorConnected = pivotMotor.getTalonFX().isConnected();

    inputs.rollerMotorDisabled = isRollerMotorDisabled;
    inputs.pivotMotorDisabled = isPivotMotorDisabled;

    BaseStatusSignal.refreshAll(pivotMotorPosition, rollerMotorAngularVelocity);

    BaseStatusSignal.refreshAll(
        pivotMotorAngularAcceleration,
        pivotMotorAngularVelocity,
        rollerMotorAngularAcceleration,
        rollerMotorCurrent,
        pivotMotorCurrent);

    BaseStatusSignal.refreshAll(
        pivotMotorTemperature, pivotMotorVoltage, rollerMotorTemperature, rollerMotorVoltage);

    inputs.pivotMotorRotation =
        Rotation2d.fromRotations(pivotMotorPosition.getValue().in(Rotation));
    inputs.pivotMotorAngularVelocity = pivotMotorAngularVelocity.getValue();
    inputs.pivotMotorAngularAcceleration = pivotMotorAngularAcceleration.getValue();
    inputs.pivotMotorCurrent = pivotMotorCurrent.getValue();
    inputs.pivotMotorVoltage = pivotMotorVoltage.getValue();
    inputs.pivotMotorTemperature = pivotMotorTemperature.getValue();

    inputs.rollerMotorAngularVelocity = rollerMotorAngularVelocity.getValue();
    inputs.rollerMotorAngularAcceleration = rollerMotorAngularAcceleration.getValue();
    inputs.rollerMotorCurrent = rollerMotorCurrent.getValue();
    inputs.rollerMotorLinearVelocity =
        MetersPerSecond.of(rollerMotor.getEncoderVelocity(rollerMotorAngularVelocity));
    inputs.rollerMotorLinearAcceleration =
        MetersPerSecondPerSecond.of(
            rollerMotor.getEncoderAcceleration(rollerMotorAngularAcceleration));
    inputs.rollerMotorTemperature = rollerMotorTemperature.getValue();
    inputs.rollerMotorVoltage = rollerMotorVoltage.getValue();
  }

  public void setPivotRotation(Rotation2d rotation) {
    if (!isPivotMotorDisabled) {
      pivotMotor.setSetpoint(rotation);
    }
  }

  public void setPivotRotation(Angle rotation) {
    if (!isPivotMotorDisabled) {
      pivotMotor.setSetpoint(rotation);
    }
  }

  public void setRollerSpeed(LinearVelocity velocity) {
    if (!isRollerMotorDisabled) {
      rollerMotor.setSetpoint(velocity);
    }
  }

  public void setRollerSpeed(AngularVelocity velocity) {
    if (!isRollerMotorDisabled) {
      rollerMotor.setSetpoint(velocity);
    }
  }

  public void disableRollerMotor(boolean shouldDisable) {
    isRollerMotorDisabled = shouldDisable;
    if (shouldDisable) rollerMotor.setRawPercentage(0);
  }

  public void disablePivotMotor(boolean shouldDisable) {
    isPivotMotorDisabled = shouldDisable;
    if (shouldDisable) pivotMotor.setRawPercentage(0);
  }
}
