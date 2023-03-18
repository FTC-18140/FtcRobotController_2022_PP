package org.firstinspires.ftc.teamcode.Commands.unused;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s).
 */
public class DefaultDriveCommand extends CommandBase
{
    private final ChassisSubsystem myChassis;
    private final DoubleSupplier myForwardSupplier;
    private final DoubleSupplier myRotationSupplier;
    public static double NON_TURBO_FACTOR = 0.45;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param strafe    Not used
     * @param rotation  The control input for turning
     */
    public DefaultDriveCommand(ChassisSubsystem subsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation)
    {
        myChassis = subsystem;
        myForwardSupplier = forward;
        myRotationSupplier = rotation;
        addRequirements(myChassis);
    }

    @Override
    public void execute()
    {
        myChassis.arcadeDrive(myForwardSupplier.getAsDouble() * NON_TURBO_FACTOR,
                              myRotationSupplier.getAsDouble() * 0.2);
    }
}
