
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.GrabberHandling;
import org.firstinspires.ftc.teamcode.drive.RingHandling;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PersistentStorage;

import java.util.Arrays;
import java.util.List;

/* This is the top-level class for the tele operated period, a large portion of robot control
 * systems are implemented in multiple lower level classes. In this class all that is left to do is
 * to initialize all the lower level things and assign actions to their gamepad buttons.
 * This program is not perfect in that there are still a few more complicated functions being done
 * here for example multiple nested if else statements which would be better suited for a separate class.
 *
 * We use an iterative op-mode structure for our tele-op since this nicely structures everything
 * according to the phases in the program's execution. See the sa
 */
@Config
@TeleOp(name="TeleOp Field Centric", group="basic")
public class TeleOpFieldCentric extends OpMode
{
    // Declare op mode members and helping variables
    SampleMecanumDrive drive;
    RingHandling rings;
    GrabberHandling grabber;
    Servo blocker;

    RevBlinkinLedDriver blinkin;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    RevBlinkinLedDriver.BlinkinPattern prevPattern;

    ElapsedTime rotateTimer = new ElapsedTime();
    ElapsedTime matchTimer = new ElapsedTime();

    boolean wasRotating;
    double rotationSetpoint;
    double currentHeading;
    double rotationError;
    double rotate;
    final double adjustmentSpeed = 0.4;

    boolean grabberClosed;

    private int shootingStep;
    String target;
    Pose2d poseOffset;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initializing the robot classes has you pass through hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        rings = new RingHandling(hardwareMap);
        grabber = new GrabberHandling(hardwareMap);

        // Robot hardware initialized here uses this structure
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.setPosition(0.75);

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);

        // We use a stored value for our starting pose, either a default or from a previous opmode saving it
        drive.setPoseEstimate(PersistentStorage.currentPose);

        // Giving certain helping variables a valid starting value
        wasRotating = false;
        rotationSetpoint = currentHeading = PersistentStorage.currentPose.getHeading();
        poseOffset = new Pose2d(0, 0, 0);

        shootingStep = -1;
        target = "red high";

        grabber.moveGrabber("inSize", "closed");
        grabberClosed = true;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        drive.update(); // We track our position whenever possible
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        matchTimer.reset();

        pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW;
        blinkin.setPattern(pattern);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Pose2d poseEstimate = drive.getPoseEstimate(); // Pose estimate is often called so we store it separately

        // Create a vector from gamepad x/y, then rotate it by current robot heading.
        // If applicable, use offset to change field centric orientation.
        Vector2d input = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y).times(0.3 + 0.7 * gamepad1.right_trigger)
                .rotated((-poseEstimate.getHeading()) - poseOffset.getHeading());

        currentHeading = poseEstimate.getHeading();

        // This block steers the robot towards a given rotation setpoint to counteract any drift
        if (Math.abs(rotationSetpoint - currentHeading) < Math.PI) {
            rotationError = rotationSetpoint - currentHeading;
        } // We need multiple statements to deal with the rollover of the gyros range of -180 to 180 degrees
        else if (rotationSetpoint - currentHeading > 0) {
            rotationError = (rotationSetpoint - currentHeading) - 2 * Math.PI;
        }
        else {
            rotationError = 2 * Math.PI + (rotationSetpoint - currentHeading);
        }

        // Does everything around when the driver decides to rotate the robot
        if (gamepad1.right_stick_x == 0 && !wasRotating) { // Without any input we just do the little adjustments
            rotate = adjustmentSpeed * (rotationError);
            if (rotate >= 0) {
                rotate = Math.pow(rotate, 0.6);
            }
            else {
                rotate = -Math.pow(-rotate, 0.6);
            }
        } else if (wasRotating && gamepad1.right_stick_x == 0) { // When input has stopped since the last cycle
            // We give a couple tenths of a second for the robot to settle before starting the adjustment
            if (rotateTimer.milliseconds() > 300) {
                wasRotating = false;
            }
            rotationSetpoint = currentHeading;
            rotate = 0;
        } else if (!wasRotating) { // This happens when there is new input
            wasRotating = true;
            rotate = -gamepad1.right_stick_x * (0.3 + 0.7 * gamepad1.right_trigger);
            rotateTimer.reset();
        } else { // For ongoing rotation input
            rotate = -gamepad1.right_stick_x * (0.3 + 0.7 * gamepad1.right_trigger);
            rotateTimer.reset();
        }

        if (Math.abs(rotate) < 0.05) {
            rotate = 0; // Prevent the motors from receiving so little power they can't overcome static friction
        }
        // Pass rotated input + right stick value for rotation to drive function
        drive.setDrivePower(new Pose2d(input.getX(), input.getY(), rotate));

        // Shooting the high goal automatically
        if (gamepad1.left_bumper) {
            if (rings.getRingNumber() != 0) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;

                double calcHeading = rings.shootGetHeading(poseEstimate, target);
                // Dealing with the gyro range of -180 to 180 is a pain
                if (Math.abs(calcHeading - currentHeading) < Math.PI) {
                    drive.turn(calcHeading - currentHeading);
                }
                else if (calcHeading - currentHeading > 0){
                    drive.turn((calcHeading - currentHeading) - 2 * Math.PI);
                }
                else {
                    drive.turn(2 * Math.PI + (calcHeading - currentHeading));
                }
                blocker.setPosition(0.14);
                rings.shoot();
                wasRotating = true;
                rotateTimer.reset();
            }
        }
        else if (gamepad1.back) {
            rings.stopShooting();
            blocker.setPosition(0.5);
        }

        // Shooting three powershots automatically
        if (gamepad1.left_trigger == 1) {
            Pose2d shootPose = new Pose2d(-2, -25, 0); // Where the robot should shoot from
            // Get the robot in position
            Vector2d headingVector = new Vector2d(poseEstimate.minus(shootPose).getX(), poseEstimate.minus(shootPose).getY());
            Trajectory shootPower = drive.trajectoryBuilder(poseEstimate, headingVector.angle())
                    .splineToSplineHeading(shootPose, headingVector.angle())
                    .build();
            drive.followTrajectory(shootPower);
            wasRotating = true;
            // Signal to the other block that we wish to shoot
            shootingStep = 0;
        }
        if (shootingStep != -1) { // -1 Means we aren't shooting, anything else makes this trigger
            List<String> targetsList = Arrays.asList("red right", "red mid", "red left");
            // This is called a state machine, this one is a bit weird since multiple cases have the same state
            switch (shootingStep) {
                case 0: case 2: case 4: // Here we start the shooting process
                    double calcHeading = rings.shootGetHeading(poseEstimate, targetsList.get(shootingStep/2));
                    double currentHeading = poseEstimate.getHeading();
                    // Aiming
                    if (Math.abs(calcHeading - currentHeading) < Math.PI) {
                        drive.turn(calcHeading - currentHeading);
                    }
                    else if (calcHeading - currentHeading > 0){
                        drive.turn((calcHeading - currentHeading) - 2 * Math.PI);
                    }
                    else {
                        drive.turn(2 * Math.PI + (calcHeading - currentHeading));
                    }
                    wasRotating = true;
                    // Shooting
                    target = targetsList.get(shootingStep/2);
                    rings.shoot();
                    // Moving to the wait state
                    shootingStep++;
                    break;
                case 1: case 3: case 5: // Here we wait until the ring has been shot and then move to the next step
                    if (rings.state_s == RingHandling.shooterStates.NOTHING) {
                        shootingStep++;
                    }
                    break;
                case 6: // When all rings are done we reset
                    shootingStep = -1;
                    target = "red high";
                    break;
            }

        }

        // An unreliable way to take the blocker away when the shooting has stopped
        if (rings.state_s == RingHandling.shooterStates.NOTHING && pattern == RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW;
            blocker.setPosition(0.5);
        }

        if (gamepad2.right_bumper) { // Resetting the tracked position is on gamepad 2 to avoid accidents
            drive.setPoseEstimate(new Pose2d(-62, -63.8, Math.PI));
            PersistentStorage.currentPose = new Pose2d(-62, -63.8, Math.PI);
            wasRotating = true;
        }

        // Intake control
        if (gamepad1.a) {
            rings.setIntake(1);
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST;
        }
        else if (gamepad1.y) {
            rings.setIntake(0);
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW;
        }
        else if (gamepad1.x) {
            rings.setIntake(-1);
        }

        // Grabber arm and gripper control
        if (gamepad1.dpad_down) {
            grabber.moveGrabber("down", "");
        }
        else if (gamepad1.dpad_up && grabberClosed) {
            grabber.moveGrabber("upHalf", "");
        }
        else if (gamepad1.dpad_up) {
            grabber.moveGrabber("inSize", "closed");
        }
        if (gamepad1.dpad_left) {
            grabber.moveGrabber("", "closed");
            grabberClosed = true;
        }
        else if (gamepad1.dpad_right) {
            grabber.moveGrabber("", "open");
            grabberClosed = false;
        }

        // Choosing different targets to shoot at
        if (gamepad1.right_stick_y == -1) { // up on joystick
            target = "red high";
        }
        else if (gamepad1.right_stick_y == 1) { // down on joystick
            target = "red mid";
        }
        else if (gamepad1.left_stick_button) {
            target = "red left";
        }
        else if (gamepad1.right_stick_button) {
            target = "red right";
        }

        if (pattern != prevPattern) { // Set the blinkin led pattern when it has changed
            blinkin.setPattern(pattern);
            prevPattern = pattern;
        }

        // updates everything. Localizer, drive functions, etc.
        drive.update();
        rings.update(matchTimer.milliseconds(), poseEstimate, target);

        telemetry.addData("rpm", rings.getRPM());
        telemetry.addData("calcRPM", rings.calcRPM);
//        telemetry.addData("distance", rings.getDistanceSensor());
//        telemetry.addData("rings amount", rings.getRingNumber());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // When the op-mode is stopped we store the position for the next op-mode start
        PersistentStorage.currentPose = drive.getPoseEstimate();
    }

}
