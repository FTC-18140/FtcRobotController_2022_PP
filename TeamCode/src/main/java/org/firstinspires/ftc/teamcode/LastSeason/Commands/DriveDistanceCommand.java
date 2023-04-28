package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.DiffDriveOdometrySubsystem;
@Config
public class DriveDistanceCommand extends CommandBase
{
    private final ChassisSubsystem myChassisSubsystem;

    private final DiffDriveOdometrySubsystem myOdometrySubsystem;
    private double myDistance;
    private double mySpeed;
    private double myInitialDistanceLeft;
    private double myInitialDistanceRight;
    private final MotionProfile myMotionProfile = new MotionProfile(10, 10, 1, 0.02);

    public static double distanceFromPole = 23.5;
    /**
     * Creates a new DriveDistanceCommand.
     *
     * @param cm The number of centimeters the robot will drive
     * @param speed  The speed at which the robot will drive
     * @param chassis  The drive subsystem on which this command will run
     */
    public DriveDistanceCommand(double cm, double speed, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {

        myDistance = cm;
        mySpeed = speed;
        myChassisSubsystem = chassis;
        myOdometrySubsystem = odometry;
        addRequirements(myChassisSubsystem);
        addRequirements(myOdometrySubsystem);
    }

    @Override
    public void initialize() {
       myDistance = myChassisSubsystem.getFrontDistance() - distanceFromPole;
        myInitialDistanceLeft =  myChassisSubsystem.getLeftEncoderDistance();
        myInitialDistanceRight = myChassisSubsystem.getRightEncoderDistance();
    }

    @Override
    public void execute()
    {
        myChassisSubsystem.getTelemetry().addData("Executing Drive Distance Command: ", myDistance);
        myChassisSubsystem.getTelemetry().addData("Distance Left: ", myChassisSubsystem.getLeftEncoderDistance() - myInitialDistanceLeft);
        myChassisSubsystem.getTelemetry().addData("Distance Right: ", myChassisSubsystem.getRightEncoderDistance() - myInitialDistanceRight);

        double[] motorPowers = new double[]{mySpeed, 0, 0};

        myChassisSubsystem.arcadeDrive(motorPowers[0], 0);
    }

    @Override
    public void end(boolean interrupted) {
        myChassisSubsystem.stop();
    }


    @Override
    public boolean isFinished()
    {
        return (Math.abs(myChassisSubsystem.getLeftEncoderDistance() - myInitialDistanceLeft) >= myDistance) &&
               (Math.abs(myChassisSubsystem.getRightEncoderDistance() - myInitialDistanceRight ) >= myDistance);
    }

    /**
     * Adjusts the motor speeds based on this path's motion profile.
     *
     * @param speeds       Speeds to be adjusted.
     */
    private void adjustSpeedsWithProfile(double[] speeds, double robotDistance)
    {
        // If we are less than half way there, do acceleration.  Otherwise do deceleration.
        if (robotDistance < myDistance / 2.0)
        {   // If the robot is closer to the "from" point.
            myMotionProfile.processAccelerate(speeds, robotDistance, mySpeed, 0);
        }
        else
        {   // If the robot is closer to the "to" point.
            myMotionProfile.processDecelerate(speeds, myDistance - robotDistance, mySpeed, 0);
        }
        myChassisSubsystem.getTelemetry().addData("speeds: ", "%f.6, %f.6", speeds[0], speeds[1]);
    }

}
