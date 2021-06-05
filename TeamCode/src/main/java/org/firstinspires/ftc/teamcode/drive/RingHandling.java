package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;


public class RingHandling {
    DcMotorEx feeder;
    public DcMotorEx shooter;
    DcMotorEx intakeChain;
    DcMotorEx intakeSingle;
    Servo pusher;
    DistanceSensor distance;

    private VoltageSensor batteryVoltageSensor;
    PIDFCoefficients pidf = new PIDFCoefficients(60, 0.09, 0.01, 13.3);

    double pusherStart;
    double shooterTime;

    public shooterStates state_s;

    HardwareMap hardwareMap;

    public int calcRPM;

    public enum shooterStates {
        INITIALIZE, CHECKRPM, WAIT, NOTHING
    }

    public RingHandling(@NotNull HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeChain = hardwareMap.get(DcMotorEx.class, "intakeChain");
        intakeSingle = hardwareMap.get(DcMotorEx.class, "intakeSingle");
        pusher = hardwareMap.get(Servo.class, "pusher");
        distance = hardwareMap.get(DistanceSensor.class, "distance");


        pusher.setPosition(0.2);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        pidf = new PIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f * 12 / batteryVoltageSensor.getVoltage());
        shooter.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);

        state_s = shooterStates.NOTHING;

        intakeChain.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIntake(double power) {
        intakeSingle.setPower(power);
        intakeChain.setPower(power);
    }

    public double getIntake() {
        return intakeChain.getPower();
    }

    public void setRPM(double rpm) {
        // unit conversion from the input, rpm, to setVelocity's desired input, encoder ticks per second
        double ticksPerSecond = rpm / 60 * 28;
        shooter.setVelocity(-ticksPerSecond);
    }

    public double getRPM() {
        // unit conversion from the input, encoder ticks per second, to the desired output, rpm
        double ticksPerSecond = shooter.getVelocity();
        return ticksPerSecond / 28 * -60;
    }

    public double shootGetHeading (Pose2d currentPose, String target) {
        Pose2d tower;
        
        if (target.equals("red high")) {
            tower = new Pose2d(72, -38);
        }
        else if (target.equals("blue high")) {
            tower = new Pose2d(72, 36);
        }
        else if (target.equals("red left")) {
            tower = new Pose2d(72, -8);
        }
        else if (target.equals("red mid")) {
            tower = new Pose2d(72, -15);
        }
        else {
            tower = new Pose2d(72, -21);
        }

        currentPose = currentPose.minus(tower);
        Vector2d vector = new Vector2d(currentPose.getX(), currentPose.getY());
        return vector.rotated(Math.PI - Math.toRadians(4)).angle();
    }

    public double shootGetRPM (Pose2d currentPose, String target) {
        double goalHeight;
        Pose2d tower;

        if (target.equals("red high")) {
            tower = new Pose2d(72, -38);
            goalHeight = 43.5;
        }
        else if (target.equals("blue high")) {
            tower = new Pose2d(72, 36);
            goalHeight = 43.5;
        }
        else if (target.equals("red left")) {
            tower = new Pose2d(72, -8);
            goalHeight = 33.5;
        }
        else if (target.equals("red mid")) {
            tower = new Pose2d(72, -15);
            goalHeight = 33.5;
        }
        else {
            tower = new Pose2d(72, -21);
            goalHeight = 33.5;
        }

        currentPose = currentPose.minus(tower);
        Vector2d vector = new Vector2d(currentPose.getX(), currentPose.getY());
        double distance = 2.54 * vector.norm() - 20; // Distance to goal converted from inches to cm

        return (Math.sqrt((490*Math.pow(distance, 2))/(distance*Math.tan(Math.toRadians(28))-goalHeight)))/(0.15) - Math.pow(0.04 * distance, 2.55);
    }

    public double getDistance (Pose2d currentPose, String target) {
        Pose2d tower;

        if (target.equals("red high")) {
            tower = new Pose2d(72, -38);
        }
        else if (target.equals("blue high")) {
            tower = new Pose2d(72, 36);
        }
        else if (target.equals("red left")) {
            tower = new Pose2d(72, -8);
        }
        else if (target.equals("red mid")) {
            tower = new Pose2d(72, -15);
        }
        else {
            tower = new Pose2d(72, -21);
        }

        currentPose = currentPose.minus(tower);
        Vector2d vector = new Vector2d(currentPose.getX(), currentPose.getY());
        return 2.54 * vector.norm(); // Distance to goal converted from inches to cm
    }

    public void triggerPusher(double startTime) {
        pusher.setPosition(0.4);
        pusherStart = startTime;
    }

    public double getDistanceSensor() {
        return distance.getDistance(DistanceUnit.MM);
    }

    public int getRingNumber() {
        if (getDistanceSensor() < 70) {
            return 3;
        }
        else if (getDistanceSensor() < 90) {
            return 2;
        }
        else if (getDistanceSensor() < 112) {
            return 1;
        }
        else if (getDistanceSensor() > 200) {
            return 3;
        }
        else {
            return 0;
        }
    }

    public void shoot() {
        state_s = shooterStates.INITIALIZE;
    }

    public void stopShooting() {
        setRPM(0);
        state_s = shooterStates.NOTHING;
    }

    public void update(double time, Pose2d currentPose, String target) {
        if (pusherStart >= 0) {
            if (time - pusherStart >= 600) {
                pusher.setPosition(0.2);
                pusherStart = -1;
            }
        }

        switch (state_s) {
            case INITIALIZE:
                if ((getRingNumber() == 0) && !(target.contains("left") || target.contains("mid") || target.contains("right"))) {
                    state_s = shooterStates.NOTHING;
                    break;
                }
                calcRPM = (int) shootGetRPM(currentPose, target);
                setRPM(calcRPM);
                state_s = shooterStates.CHECKRPM;
                break;
            case CHECKRPM:
                if (getRPM() > (calcRPM - 60) && getRPM() < (calcRPM + 60)) {
                    if (shooterTime == -1) {
                        shooterTime = time;
                    }
                    else if (time - shooterTime >= 300) {
                        triggerPusher(time);
                        shooterTime = -1;
                        state_s = shooterStates.WAIT;
                    }
                }
                else {
                    shooterTime = -1;
                }
                break;
            case WAIT:
                if (pusherStart != -1) {
                    break;
                }
                else if (target.contains("left") || target.contains("mid") || target.contains("right")) {
                    setRPM(0);
                    state_s = shooterStates.NOTHING;
                }
                else if (getRingNumber() != 0) {
                    state_s = shooterStates.CHECKRPM;
                }
                else {
                    setRPM(0);
                    state_s = shooterStates.NOTHING;
                }
                break;
            case NOTHING:
                break;
        }
    }
}
