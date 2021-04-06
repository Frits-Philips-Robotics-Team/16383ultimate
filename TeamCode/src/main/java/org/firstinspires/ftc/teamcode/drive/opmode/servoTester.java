package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="servoTester", group="testing")
public class servoTester extends OpMode {
    private Servo servo_one;
    private Servo servo_two;
    private ElapsedTime cycleTimeOne = new ElapsedTime();
    private ElapsedTime cycleTimeTwo = new ElapsedTime();

    private double angleOne;
    private double angleTwo;

    private static int GAMEPAD_LOCKOUT = 500;
    Deadline gamepadRateLimit;

    @Override
    public void init() {
        servo_one = hardwareMap.get(Servo.class, "gripper");
        servo_two = hardwareMap.get(Servo.class, "arm");

        angleOne = 0.5;
        angleTwo = 0.5;

        cycleTimeOne.reset();
        cycleTimeTwo.reset();

        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }

    @Override
    public void loop() {
        handleGamepad();

        if (gamepad1.left_bumper) {
            GAMEPAD_LOCKOUT = 50;
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
        }
        else if (gamepad1.right_bumper) {
            GAMEPAD_LOCKOUT = 500;
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
        }

        reportPositions();
    }

    protected void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.dpad_up) {
            moveOne(true);
            gamepadRateLimit.reset();
        }
        else if (gamepad1.dpad_left) {
            moveOne(false);
            gamepadRateLimit.reset();
        }
        else if (gamepad1.y) {
            moveTwo(true);
            gamepadRateLimit.reset();
        }
        else if (gamepad1.x) {
            moveTwo(false);
            gamepadRateLimit.reset();
        }


    }

    public void moveOne(boolean up) {
        if(cycleTimeOne.milliseconds() >= 50) {
            if(up) {
                angleOne += 0.01;
            }
            else {
                angleOne -= 0.01;
            }
            servo_one.setPosition(angleOne);
        }
    }

    public void moveTwo(boolean up) {
        if(cycleTimeTwo.milliseconds() >= 50) {
            if(up) {
                angleTwo += 0.01;
            }
            else {
                angleTwo -= 0.01;
            }
            servo_two.setPosition(angleTwo);
        }
    }

    protected void reportPositions() {
        telemetry.addData("servos", "servo_one: %f, servo_two: %f", angleOne, angleTwo);
    }
}
