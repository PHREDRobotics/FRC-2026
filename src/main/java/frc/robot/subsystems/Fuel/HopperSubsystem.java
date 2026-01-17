package frc.robot.subsystems.Fuel;

import frc.robot.Configs;
import frc.robot.Configs.HopperSparkMax;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class HopperSubsystem {


  public SparkMax hopperSparkMax = new SparkMax(FuelConstants.kHopperMotorCANId, MotorType.kBrushless);


public HopperSubsystem() {
    hopperSparkMax.configure(Configs.HopperSparkMax.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

    public void stop() {
    hopperSparkMax.set(0);
  }


  public void extendHopper() {
    hopperSparkMax.set(FuelConstants.kHopperSpeed);
  }

}
