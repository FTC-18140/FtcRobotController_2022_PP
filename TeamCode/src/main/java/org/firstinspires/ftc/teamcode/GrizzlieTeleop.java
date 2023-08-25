package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LastSeason.Commands.Thunderbot_2022;

@TeleOp(name = "GrizzlieTeleop", group = "Teleop")
public class GrizzlieTeleop extends OpMode {
    Thunderbot_2022 robot = new Thunderbot_2022();

    public void init() {

    }

    public void start() {
        robot.start();
    }

    public void loop() {
        robot.joystickDrive(+gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        {
        }
        if (gamepad1.right_trigger > 0.1) ;
        robot.joystickDrive(-gamepad1.left_stick_y * 0.2, gamepad1.left_stick_x * 0.2, gamepad1.right_stick_x * 0.2);
        {
        }
        {
            double sign = Math.signum(gamepad1.right_stick_x);
            robot.joystickDrive(-gamepad1.left_stick_y * 0.6, gamepad1.left_stick_x * 0.6, gamepad1.right_stick_x * 0.8 * gamepad1.right_stick_x * sign);
        }
    }
}