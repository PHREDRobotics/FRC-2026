package frc.robot.subsystems.Fuel;

import frc.robot.Configs;
import frc.robot.Configs.FeederMotorSparkMax;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class FeederSubsystem {


  public SparkMax feederMotorSparkMax = new SparkMax(FuelConstants.kFeederMotorCANId, MotorType.kBrushless);


public FeederSubsystem() {
    feederMotorSparkMax.configure(Configs.FeederMotorSparkMax.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

    public void stop() {
    feederMotorSparkMax.set(0);
  }


  public void feed() {
    feederMotorSparkMax.set(FuelConstants.kFeederSpeed);
  }

}
