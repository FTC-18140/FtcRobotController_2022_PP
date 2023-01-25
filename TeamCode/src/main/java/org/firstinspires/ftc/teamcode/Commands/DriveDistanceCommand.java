package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class DriveDistanceCommand extends CommandBase {

    private final ChassisSubsystem myChassisSubsystem;
    private final double theDistance;
    private final double theSpeed;

    /**
     * Creates a new DriveDistanceCommand.
     *
     * @param cm The number of centimeters the robot will drive
     * @param speed  The speed at which the robot will drive
     * @param chassis  The drive subsystem on which this command will run
     */
    public DriveDistanceCommand(double cm, double speed, ChassisSubsystem chassis) {
        theDistance = cm;
        theSpeed = speed;
        myChassisSubsystem = chassis;
    }

    @Override
    public void initialize() {
        myChassisSubsystem.resetEncoders();
    }

    @Override
    public void execute()
    {
        super.execute();
        myChassisSubsystem.joystickDrive(theSpeed, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        myChassisSubsystem.stop();
    }


    @Override
    public boolean isFinished() {
        return Math.abs(myChassisSubsystem.getAverageEncoderDistance()) >= theDistance;
    }

}
