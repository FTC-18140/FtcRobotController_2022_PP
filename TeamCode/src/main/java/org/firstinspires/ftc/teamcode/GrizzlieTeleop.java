package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GrizzlieTeleop", group = "Teleop")
public class GrizzlieTeleop extends OpMode {
    Thunderbot_2022 robot = new Thunderbot_2022();

    public void init() {
        telemetry.addData("Robot", " Initialized");
        robot.init(hardwareMap, telemetry);
    }

    public void start() {
    //    robot.start();
        telemetry.addData("Robot", " Started");
    }

    public void loop() {
        telemetry.addData("Forward Stick", gamepad1.left_stick_y);
        telemetry.addData("Sideways Stick", gamepad1.left_stick_x);
        telemetry.addData("Clockwise", gamepad1.right_stick_x);
        if (gamepad1.a) {
            robot.joystickDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        } else {
            robot.joystickDrive(gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5, -gamepad1.right_stick_x * 0.5);
        }
    }
}