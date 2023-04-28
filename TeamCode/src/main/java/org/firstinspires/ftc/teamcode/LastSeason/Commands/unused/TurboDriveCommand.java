package org.firstinspires.ftc.teamcode.LastSeason.Commands.unused;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.ChassisSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s).
 */
public class TurboDriveCommand extends CommandBase
{
    private final ChassisSubsystem myChassis;
    private final DoubleSupplier myForwardSupplier;
    private final DoubleSupplier myStrafeSupplier;
    private final DoubleSupplier myRotationSupplier;

    /**
     * Creates a new TurboDriveCommand.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param strafe    Not used
     * @param rotation  The control input for turning
     */
    public TurboDriveCommand(ChassisSubsystem subsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation )
    {
        myChassis = subsystem;
        myForwardSupplier = forward;
        myRotationSupplier = rotation;
        myStrafeSupplier = strafe;
        addRequirements(myChassis);
    }

    @Override
    public void execute()
    {
        myChassis.arcadeDrive(myForwardSupplier.getAsDouble(), myRotationSupplier.getAsDouble());
    }
}