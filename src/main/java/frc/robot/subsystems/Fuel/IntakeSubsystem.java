package frc.robot.subsystems.Fuel;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem {


  public SparkMax intakeMotorSparkMax = new SparkMax(FuelConstants.kIntakeMotorCANId, MotorType.kBrushless);
  public SparkMax intakeMovementSparkMax = new SparkMax(FuelConstants.kIntakeMovementCANId, MotorType.kBrushless);


public IntakeSubsystem() {
    intakeMotorSparkMax.configure(Configs.IntakeMovementSparkMax.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeMovementSparkMax.configure(Configs.IntakeMovementSparkMax.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

    public void stop() {
    intakeMotorSparkMax.set(0);
  }


  public void startIntake() {
    intakeMotorSparkMax.set(FuelConstants.kIntakeSpeed);
  }

  public void moveIntake() {
    intakeMovementSparkMax.set(FuelConstants.kIntakeMovementSpeed);
  }

}
