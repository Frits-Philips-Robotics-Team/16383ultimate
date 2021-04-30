package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;


public class GrabberHandling {
    Servo arm;
    Servo gripper;

    public GrabberHandling(@NotNull HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");
    }


    public void moveGrabber(String armPos, String gripperPos) {
        final double upHalfValue = 0.6;
        final double inSizeValue = 0.93;
        final double straightValue = 0.26;
        final double downValue = 0.2;
        final double openValue = 1;
        final double closedValue = 0.55;

        switch (armPos) {
            case "upHalf": arm.setPosition(upHalfValue);
                break;
            case "inSize": arm.setPosition(inSizeValue);
                break;
            case "straight": arm.setPosition(straightValue);
                break;
            case "down":   arm.setPosition(downValue);
                break;
            default: break;
        }
        switch (gripperPos) {
            case "open":    gripper.setPosition(openValue);
                break;
            case "closed":  gripper.setPosition(closedValue);
                break;
            default: break;
        }
    }
}
