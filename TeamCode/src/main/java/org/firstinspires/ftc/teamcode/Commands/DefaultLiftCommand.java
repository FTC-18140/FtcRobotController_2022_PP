package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
@Config
public class DefaultLiftCommand extends CommandBase {

    private final LiftSubsystem theSubsystem;
    private boolean up = true;
    private double speed = 0.7;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param subsystem The drive subsystem this command wil run on.

     */
    public DefaultLiftCommand(boolean liftUp, double liftSpeed, LiftSubsystem subsystem)
    {
        up = liftUp;
        speed = liftSpeed;
        theSubsystem = subsystem;
        addRequirements(theSubsystem);
    }

    @Override
    public void execute()
    {
        if (up)
        {
            theSubsystem.liftUp(speed);
        }
        else
        {
            theSubsystem.liftDown(speed);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        theSubsystem.liftStop();
    }
}