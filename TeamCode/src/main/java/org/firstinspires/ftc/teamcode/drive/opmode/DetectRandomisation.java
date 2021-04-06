package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.GrabberHandling;
import org.firstinspires.ftc.teamcode.drive.RingHandling;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PersistentStorage;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name= "detectRandomisation", group="red")
//@Disabled//comment out this line before using
public class DetectRandomisation extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    SampleMecanumDrive drive;
    RingHandling rings;
    GrabberHandling grabber;

    //0 means no ring, 255 means orange ring
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valTop = -1;
    private static int valBottom = -1;
    private static float rectHeight = .6f/8f;
    private static float rectWidth = .6f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] topPos = {3f/8f+offsetX, 5f/8f+offsetY};//0 = col, 1 = row
    private static float[] bottomPos = {3f/8f+offsetX, 6f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 240;
    private final int cols = 320;

    private OpenCvCamera webcam;

    private int captureCounter = 0;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(cols, rows, OpenCvCameraRotation.UPRIGHT);//display on RC
            }
        });

        drive = new SampleMecanumDrive(hardwareMap);
        rings = new RingHandling(hardwareMap);
        grabber = new GrabberHandling(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-61, -42, Math.PI));

        Trajectory dropA = drive.trajectoryBuilder(new Pose2d(-61, -42, Math.PI), 0)
                .lineToConstantHeading(new Vector2d(12, -48))
                .build();
        Trajectory pickUpA = drive.trajectoryBuilder(dropA.end(), Math.toRadians(155))
                .splineToLinearHeading(new Pose2d(-24, -19, 0.5 * Math.PI), Math.PI)
                .build();
        Trajectory dropA2 = drive.trajectoryBuilder(new Pose2d(pickUpA.end().getX() - 7, pickUpA.end().getY(), 0.5 * Math.PI), 0)
                .splineToLinearHeading(new Pose2d(12, -40, Math.PI), 0)
                .build();
        Trajectory shootA = drive.trajectoryBuilder(dropA2.end(), Math.PI)
                .splineToLinearHeading(new Pose2d(-4, -37, 0), Math.PI)
                .build();
        Trajectory parkA = drive.trajectoryBuilder(shootA.end(), 0)
                .splineToLinearHeading(new Pose2d(12, shootA.end().getY(), 0), 0)
                .build();

        Trajectory dropB = drive.trajectoryBuilder(new Pose2d(-61, -42, Math.PI), 0)
                .lineToConstantHeading(new Vector2d(36, -30))
                .build();
        Trajectory pickUpB = drive.trajectoryBuilder(dropB.end(), Math.PI)
                .splineToLinearHeading(new Pose2d(-24, -19, 0.5 * Math.PI), Math.PI)
                .build();
        Trajectory dropB2 = drive.trajectoryBuilder(new Pose2d(pickUpB.end().getX() - 7, pickUpB.end().getY(), 0.5 * Math.PI), 0)
                .splineToLinearHeading(new Pose2d(36, -19, Math.PI), 0)
                .build();
        Trajectory shootB = drive.trajectoryBuilder(dropB2.end(), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-4, -30, 0), Math.PI)
                .build();
        Trajectory parkB = drive.trajectoryBuilder(shootB.end(), 0)
                .splineToLinearHeading(new Pose2d(12, shootB.end().getY(), 0), 0)
                .build();

        Trajectory dropC = drive.trajectoryBuilder(new Pose2d(-61, -42, Math.PI), 0)
                .lineToConstantHeading(new Vector2d(60, -49))
                .build();
        Trajectory pickUpC = drive.trajectoryBuilder(dropC.end(), Math.toRadians(155))
                .splineToLinearHeading(new Pose2d(-24, -18, 0.5 * Math.PI), Math.PI)
                .build();
        Trajectory dropC2 = drive.trajectoryBuilder(new Pose2d(pickUpC.end().getX() - 7, pickUpC.end().getY(), 0.5 * Math.PI), 0)
                .splineToLinearHeading(new Pose2d(60, -41, Math.PI), 0)
                .build();
        Trajectory shootC = drive.trajectoryBuilder(dropC2.end(), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-4, -37, 0), Math.PI)
                .build();
        Trajectory parkC = drive.trajectoryBuilder(shootC.end(), 0)
                .splineToLinearHeading(new Pose2d(12, shootC.end().getY(), 0), 0)
                .build();

        Trajectory grabWobble = drive.trajectoryBuilder(pickUpA.end(), Math.PI)
                .strafeRight(-7)
                .build();

        telemetry.addData("Values", valBottom + "   " + valTop);
        telemetry.addData("Height", rows);
        telemetry.addData("Width", cols);
        telemetry.update();

        grabber.moveGrabber("inSize", "closed");

        while (!isStarted()) {
            telemetry.update();
        }

        waitForStart();
        runtime.reset();

        grabber.moveGrabber("upHalf", "closed");

        if (valBottom == 0 && valTop == 0) { // No rings; target zone A
            drive.followTrajectory(dropA);
            grabber.moveGrabber("down", "closed");
            sleep(300);
            grabber.moveGrabber("down", "open");
            sleep(500);
            grabber.moveGrabber("upHalf", "open");
            drive.followTrajectory(pickUpA);
            grabber.moveGrabber("down", "open");
            sleep(200);
            drive.followTrajectory(grabWobble);
            grabber.moveGrabber("down", "closed");
            sleep(300);
            grabber.moveGrabber("upHalf", "closed");
            sleep(400);
            drive.followTrajectory(dropA2);
            grabber.moveGrabber("down", "closed");
            sleep(300);
            grabber.moveGrabber("down", "open");
            sleep(600);
            grabber.moveGrabber("upHalf", "open");
            drive.followTrajectory(shootA);
            grabber.moveGrabber("inSize", "closed");
            double calcHeading = rings.shootGetHeading(drive.getPoseEstimate(), "red high");
            double currentHeading = drive.getPoseEstimate().getHeading();
            if (Math.abs(calcHeading - currentHeading) < Math.PI) {
                drive.turn(calcHeading - currentHeading);
            }
            else if (calcHeading - currentHeading > 0){
                drive.turn((calcHeading - currentHeading) - 2 * Math.PI);
            }
            else {
                drive.turn(2 * Math.PI + (calcHeading - currentHeading));
            }
            rings.shoot();
            PersistentStorage.currentPose = drive.getPoseEstimate(); // fail safes for if opmode exits before finishing
            while(opModeIsActive() && rings.state_s != RingHandling.shooterStates.NOTHING) {
                rings.update(runtime.milliseconds(), drive.getPoseEstimate(), "red high");
            }
            rings.update(runtime.milliseconds(), drive.getPoseEstimate(), "red high");
            PersistentStorage.currentPose = drive.getPoseEstimate(); // fail safes for if opmode exits before finishing
            drive.followTrajectory(parkA);
            PersistentStorage.currentPose = drive.getPoseEstimate();
        }
        else if (valTop == 255) { // All rings; target zone C
            drive.followTrajectory(dropC);
            grabber.moveGrabber("down", "closed");
            sleep(300);
            grabber.moveGrabber("down", "open");
            sleep(600);
            grabber.moveGrabber("upHalf", "open");
            drive.followTrajectory(pickUpC);
            grabber.moveGrabber("down", "open");
            sleep(200);
            drive.followTrajectory(grabWobble);
            grabber.moveGrabber("down", "closed");
            sleep(300);
            grabber.moveGrabber("upHalf", "closed");
            sleep(400);
            drive.followTrajectory(dropC2);
            grabber.moveGrabber("down", "closed");
            sleep(300);
            grabber.moveGrabber("down", "open");
            sleep(600);
            grabber.moveGrabber("upHalf", "open");
            drive.followTrajectory(shootC);
            grabber.moveGrabber("inSize", "closed");
            double calcHeading = rings.shootGetHeading(drive.getPoseEstimate(), "red high");
            double currentHeading = drive.getPoseEstimate().getHeading();
            if (Math.abs(calcHeading - currentHeading) < Math.PI) {
                drive.turn(calcHeading - currentHeading);
            }
            else if (calcHeading - currentHeading > 0){
                drive.turn((calcHeading - currentHeading) - 2 * Math.PI);
            }
            else {
                drive.turn(2 * Math.PI + (calcHeading - currentHeading));
            }
            rings.shoot();
            PersistentStorage.currentPose = drive.getPoseEstimate(); // fail safes for if opmode exits before finishing
            while(opModeIsActive() && rings.state_s != RingHandling.shooterStates.NOTHING) {
                rings.update(runtime.milliseconds(), drive.getPoseEstimate(), "red high");
            }
            rings.update(runtime.milliseconds(), drive.getPoseEstimate(), "red high");
            PersistentStorage.currentPose = drive.getPoseEstimate(); // fail safes for if opmode exits before finishing
            drive.followTrajectory(parkC);
            PersistentStorage.currentPose = drive.getPoseEstimate();
        }
        else { // One ring; target zone B
            drive.followTrajectory(dropB);
            grabber.moveGrabber("down", "closed");
            sleep(300);
            grabber.moveGrabber("down", "open");
            sleep(650);
            grabber.moveGrabber("upHalf", "open");
            drive.followTrajectory(pickUpB);
            grabber.moveGrabber("down", "open");
            sleep(200);
            drive.followTrajectory(grabWobble);
            grabber.moveGrabber("down", "closed");
            sleep(300);
            grabber.moveGrabber("upHalf", "closed");
            sleep(400);
            drive.followTrajectory(dropB2);
            grabber.moveGrabber("down", "closed");
            sleep(300);
            grabber.moveGrabber("down", "open");
            sleep(600);
            grabber.moveGrabber("upHalf", "open");
            drive.followTrajectory(shootB);
            grabber.moveGrabber("inSize", "closed");
            double calcHeading = rings.shootGetHeading(drive.getPoseEstimate(), "red high");
            double currentHeading = drive.getPoseEstimate().getHeading();
            if (Math.abs(calcHeading - currentHeading) < Math.PI) {
                drive.turn(calcHeading - currentHeading);
            }
            else if (calcHeading - currentHeading > 0){
                drive.turn((calcHeading - currentHeading) - 2 * Math.PI);
            }
            else {
                drive.turn(2 * Math.PI + (calcHeading - currentHeading));
            }
            rings.shoot();
            PersistentStorage.currentPose = drive.getPoseEstimate(); // fail safes for if opmode exits before finishing
            while(opModeIsActive() && rings.state_s != RingHandling.shooterStates.NOTHING) {
                rings.update(runtime.milliseconds(), drive.getPoseEstimate(), "red high");
            }
            rings.update(runtime.milliseconds(), drive.getPoseEstimate(), "red high");
            PersistentStorage.currentPose = drive.getPoseEstimate(); // fail safes for if opmode exits before finishing
            drive.followTrajectory(parkB);
            PersistentStorage.currentPose = drive.getPoseEstimate();
        }

        PersistentStorage.currentPose = drive.getPoseEstimate();
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }


            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the ring from
             * the Ultimate Goal season
             */

            //color diff cb.
            //lower cb = more blue = field = white
            //higher cb = less blue = orange ring = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 105, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixTop = thresholdMat.get((int)(input.rows()* topPos[1]), (int)(input.cols()* topPos[0]));//gets value at circle
            valTop = (int)pixTop[0];

            double[] pixBottom = thresholdMat.get((int)(input.rows()* bottomPos[1]), (int)(input.cols()* bottomPos[0]));//gets value at circle
            valBottom = (int)pixBottom[0];


            //create three points
            Point pointTop = new Point((int)(input.cols()* topPos[0]), (int)(input.rows()* topPos[1]));
            Point pointBottom = new Point((int)(input.cols()* bottomPos[0]), (int)(input.rows()* bottomPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointTop,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointBottom,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(bottomPos[0]-rectWidth/2),
                            input.rows()*(bottomPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(bottomPos[0]+rectWidth/2),
                            input.rows()*(bottomPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(topPos[0]-rectWidth/2),
                            input.rows()*(topPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(topPos[0]+rectWidth/2),
                            input.rows()*(topPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}