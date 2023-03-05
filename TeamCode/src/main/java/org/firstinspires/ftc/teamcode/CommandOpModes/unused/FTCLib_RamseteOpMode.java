package org.firstinspires.ftc.teamcode.CommandOpModes.unused;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandOpModes.TBDOpModeBase;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;

import java.util.ArrayList;
@Disabled
@Autonomous(name = "FTCLib_RamseteOpMode", group = "FTCLib")
public class FTCLib_RamseteOpMode extends TBDOpModeBase
{
    ChassisSubsystem myChassis = null;
    DiffOdometrySubsystem myOdometry = null;
    public static double b = 5;
    public static double zeta = 0.5;
    public static double maxVelocity = 48;
    public static double maxAccel = 48;

    @Override
    public void init()
    {
        try
        {
            myChassis = new ChassisSubsystem(hardwareMap, telemetry);
            register(myChassis);

            myOdometry = new DiffOdometrySubsystem(myChassis::getLeftEncoderDistance,
                                                   myChassis::getRightEncoderDistance,
                                                   telemetry);
            register(myOdometry);

            // Create Trajectory

            // Start and End points.
            Pose2d trajStart = new Pose2d(inchesToMeters(0), inchesToMeters(0), Rotation2d.fromDegrees(0));
            Pose2d trajEnd   = new Pose2d(inchesToMeters(48), inchesToMeters(0), Rotation2d.fromDegrees(0));

            // Make intermediate or "interior" points between the start and end.
            ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
            interiorWaypoints.add(new Translation2d(inchesToMeters(12), inchesToMeters(0)));
            interiorWaypoints.add(new Translation2d(inchesToMeters(24), inchesToMeters(0)));
            interiorWaypoints.add(new Translation2d(inchesToMeters(36), inchesToMeters(0)));

            // Specify the maximum velocity and acceleration allowed when following the trajectory
            TrajectoryConfig config = new TrajectoryConfig(inchesToMeters(maxVelocity), inchesToMeters(maxAccel));

            // Generate the trajectory!
            Trajectory theTrajectory = TrajectoryGenerator.generateTrajectory(trajStart, interiorWaypoints, trajEnd, config);

            // Make a new Ramsete controller
            RamseteController theController = new RamseteController(b, zeta);

            // Make a Differential Drive Kinematics object to help with the robot-centric and
            // field-centric needs of trajectory following
            DifferentialDriveKinematics theDiffDDriveKinematics = new DifferentialDriveKinematics(myOdometry.TRACK_WIDTH_METERS);

            // Build the RamseteCommand using the Trajectory, the starting pose, the Controller object,
            // the Kinematics object, and the method which will receive the computed motor velocities
            // and actually drive the robot
            RamseteCommand followTrajectory = new RamseteCommand(theTrajectory,
                                                                 myOdometry::getPose,
                                                                 theController,
                                                                 theDiffDDriveKinematics,
                                                                 myChassis::tankDrive);

            // Schedule this command to run with the scheduler
            schedule(followTrajectory);
        }
        catch (Exception e)
        {
            telemetry.addData("Something did not initialize properly.", 0);
            telemetry.addData("Ugh: ", "%s, %s, %s", e.getStackTrace()[1], e.getStackTrace()[2],
                              e.getStackTrace()[3]);
        }
    }


    @Override
    public void init_loop()
    {
        if (myOdometry != null)
        {
            myOdometry.update();
        }
        telemetry.addData("Init Loop is running... ", 1);
        telemetry.addData("X:  ", myOdometry.getPose().getX());
        telemetry.addData("Y:  ", myOdometry.getPose().getY());
        telemetry.addData("HDG:  ", myOdometry.getPose().getHeading());
    }

    @Override
    public void start()
    {
    }

    @Override
    public void stop()
    {
        super.stop();
        myChassis.stop();
    }

    private double inchesToMeters(double inches)
    {
        return inches * 2.54 / 100.0;
    }
}
