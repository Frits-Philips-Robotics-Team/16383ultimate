package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class Padvinder extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-64, -62, 0);

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-64, -62, 0))
                .splineTo(new Vector2d(0, -48), 0)
                .splineTo(new Vector2d(0, 0), Math.toRadians(90))
                .build();

        waitForStart();

        drive.followTrajectory(traj);
    }
}
