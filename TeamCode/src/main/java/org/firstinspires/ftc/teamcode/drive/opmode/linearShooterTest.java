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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.RingHandling;
import org.firstinspires.ftc.teamcode.util.PersistentStorage;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Config
@TeleOp(name="linearShooterTest", group="basic")
public class linearShooterTest extends LinearOpMode {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0 , 0, 0);

    RingHandling rings;
    ElapsedTime rotateTimer = new ElapsedTime();
    ElapsedTime gameTimer = new ElapsedTime();

    private VoltageSensor batteryVoltageSensor;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    boolean wasRotating;
    double rotationSetpoint;
    double currentHeading;
    double rotationError;
    double rotate;
    final double adjustmentSpeed = 0.4;
    ElapsedTime matchTimer = new ElapsedTime();

    double lastKp;
    double lastKi;
    double lastKd;
    double lastKf;

    Pose2d poseOffset;

    double rpmSetpoint;

    double calcRPM;


    @Override
    public void runOpMode() {
        rings = new RingHandling(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        lastKf = 0;
        lastKp = 0;
        lastKi = 0;
        lastKd = 0;

        //MOTOR_VELO_PID = new PIDFCoefficients(lastKp, lastKi, lastKd, lastKf);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(rings.shooter, MOTOR_VELO_PID);

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        drive.setPoseEstimate(PersistentStorage.currentPose);

        wasRotating = false;
        rotationSetpoint = currentHeading = PersistentStorage.currentPose.getHeading();
        poseOffset = new Pose2d(0, 0, 0);
        rpmSetpoint = 6000;

        waitForStart();

        matchTimer.reset();

        while (opModeIsActive()) {

            Pose2d poseEstimate = PersistentStorage.currentPose;

            // Broadly setting rpm with trigger and finetuning using dpad
            if (gamepad2.right_bumper) {
                rpmSetpoint = gamepad2.right_trigger * 6000;
            }
            else if (gamepad2.dpad_down) {
                rpmSetpoint -= 10;
            }
            else if (gamepad2.dpad_up) {
                rpmSetpoint += 10;
            }

            // Shooter disable and enable
            if (gamepad2.y) {
                rings.setRPM(0);
            }
            else if (gamepad2.x) {
                rings.setRPM(rpmSetpoint);
            }

            if (gamepad2.left_bumper) {
                rings.triggerPusher(matchTimer.milliseconds());
            }

            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(rings.shooter, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            // updates everything. Localizer, drive functions, etc.
//        drive.update();
            rings.update(matchTimer.milliseconds(), poseEstimate, "red");
            // This is what the shooter test is all about
            //telemetry.addData("Set RPM: ", (int) rpmSetpoint);
            telemetry.addData("Current RPM", (int) rings.getRPM());

            telemetry.addData("set rpm", rpmSetpoint);
//        telemetry.addData("distance sensor", rings.getDistanceSensor());
            PIDFCoefficients pidf = rings.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("PIDF", "%f %f %f %f", pidf.p, pidf.i, pidf.d, pidf.f);
            telemetry.addData("setPIDF", "%f %f %f %f", MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f);

            telemetry.addData("upperBound", 5000);
            telemetry.addData("lowerBound", 0);

            telemetry.update();
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setVelocityPIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
    }

    public static double getMotorVelocityF() {
        final double maxRPM = 4500;
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / (maxRPM / 60 * 28);
    }
}

