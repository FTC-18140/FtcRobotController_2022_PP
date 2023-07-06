package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Thunderbot_2022;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class RiverTestSquare extends LinearOpMode {
    Thunderbot_2022 robot = new Thunderbot_2022();

    public final int DIST = 35;
    SampleMecanumDrive drive = null;

    boolean movedSlide = false;


    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);


        Trajectory square1 = drive.trajectoryBuilder(new Pose2d(0,1, 0))
                .lineToSplineHeading(new Pose2d(DIST, 0, 90))
                .addDisplacementMarker(5, () -> {
                    telemetry.addData("Marker", "success");
                    while(!movedSlide) {
                        movedSlide = robot.armstrong.liftUpDistance(20, 0.4);
                    }
                })
                .build();

        Trajectory square2 = drive.trajectoryBuilder(square1.end())
                .lineToSplineHeading(new Pose2d(DIST, DIST, 180))
                .build();

        Trajectory square3 = drive.trajectoryBuilder(square2.end())
                .lineToSplineHeading(new Pose2d(0, DIST, -90))
                .build();

        Trajectory square4 = drive.trajectoryBuilder(square3.end())
                .lineToSplineHeading(new Pose2d(0, 0, 0))
                .build();

        waitForStart();


        drive.followTrajectory(square1);
        //drive.turn(Math.toRadians(90));

        drive.followTrajectory(square2);
        //drive.turn(Math.toRadians(90));

        drive.followTrajectory(square3);
        //drive.turn(Math.toRadians(90));

        drive.followTrajectory(square4);
        //drive.turn(Math.toRadians(90));
        telemetry.addData("slide", movedSlide);
    }

}
