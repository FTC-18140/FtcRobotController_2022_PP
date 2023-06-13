package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Thunderbot_2022;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class RiverTestSquare extends LinearOpMode {
    Thunderbot_2022 robot = new Thunderbot_2022();
    public final int DIST = 20;
    SampleMecanumDrive drive = null;



    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);


        Trajectory square1 = drive.trajectoryBuilder(new Pose2d(0,1, 0))
                .forward(DIST)
                .build();

        Trajectory square2 = drive.trajectoryBuilder(square1.end())
                .forward(DIST)
                .build();

        Trajectory square3 = drive.trajectoryBuilder(square2.end())
                .forward(DIST)
                .build();

        Trajectory square4 = drive.trajectoryBuilder(square3.end())
                .forward(DIST)
                .build();

        waitForStart();


        drive.followTrajectory(square1);
        drive.turn(Math.toRadians(135));

        drive.followTrajectory(square2);
        drive.turn(Math.toRadians(-135));

        drive.followTrajectory(square3);
        drive.turn(Math.toRadians(-135));

        drive.followTrajectory(square4);
        drive.turn(Math.toRadians(135));

    }
}
