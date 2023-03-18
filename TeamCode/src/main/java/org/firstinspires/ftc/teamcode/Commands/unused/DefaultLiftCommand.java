package org.firstinspires.ftc.teamcode.Commands.unused;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultLiftCommand extends CommandBase
{
    private final LiftSubsystem myLift;
    private boolean upDirection = true;
    private double mySpeed = 0.7;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param lift The hardware this command wil run on.

     */
    public DefaultLiftCommand(boolean liftUp, double liftSpeed, LiftSubsystem lift)
    {
        upDirection = liftUp;
        mySpeed = liftSpeed;
        myLift = lift;
        addRequirements(myLift);
    }

    @Override
    public void execute()
    {
        if (upDirection)
        {
            myLift.liftUp(mySpeed);
        }
        else
        {
            myLift.liftDown(mySpeed);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        myLift.liftStop();
    }
}