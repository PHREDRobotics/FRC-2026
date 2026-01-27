package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoShootCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;
    private SwerveSubsystem m_swerveSubsystem;
    private VisionSubsystem m_visionSubsystem;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private boolean shoot;

    public AutoShootCommand(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem,
            VisionSubsystem visionSubsystem, DoubleSupplier x, DoubleSupplier y, BooleanSupplier shoot) {
        m_shooterSubsystem = shooterSubsystem;
        m_swerveSubsystem = swerveSubsystem;
        m_visionSubsystem = visionSubsystem;
        this.x = x;
        this.y = y;
        this.shoot = shoot.getAsBoolean();

        addRequirements(shooterSubsystem, swerveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        new AlignTagCommand(m_swerveSubsystem, m_visionSubsystem, x, y);
    }

    @Override
    public void execute() {
        if (shoot) {
            //new ShootCommand(m_shooterSubsystem, m_swerveSubsystem.getShootPower()); SHOOOOT
        }
    }
}

/*
 * Auto shoot pseudo-code
 * 
 * power = k * distance^n
 * 
 * n = 1.5?
 * 
 * k = max_power / max_distance
 * 
 * k = 1/5?
 * 
 * shoot(power)
 */
