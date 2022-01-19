package org.firstinspires.ftc.teamcode.autonomous.detection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.cases.Case1;
import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "AutoCollect", group="Autonom Joc")
//@Disabled//comment out this line before using
public class FrenzyDetection extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = 0;
    private static int valLeft = 0;
    private static int valRight = 0;
    private static int valMidLow = 0;
    private static int valLeftLow = 0;
    private static int valRightLow = 0;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.4f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    public static double[] midPos = {4/8.0+offsetX, 2.3/8.0+offsetY};//0 = col, 1 = row
    public static double[] leftPos = {1/8.0+offsetX, 2.3/8.0+offsetY}; // era 2
    public static double[] rightPos = {6.3/8.0+offsetX, 2.3/8.0+offsetY}; // era 6
    //moves all rectangles right or left by amount. units are in ratio to monitor
    public static double[] midPosLow = {4/8.0+offsetX, 6/8.0+offsetY};//0 = col, 1 = row
    public static double[] leftPosLow = {0.4/8.0+offsetX, 6/8.0+offsetY}; // era 2
    public static double[] rightPosLow = {7.6/8.0+offsetX, 6/8.0+offsetY}; // era 6

    private final int rows = 1280; //640 x 480
    private final int cols = 720;

    OpenCvCamera phoneCam;

    RobotDefinition robot;
    SampleMecanumDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        //func.init(hardwareMap);

        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initialize();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            mecanumDrive.update();

            /*
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("LValues", valLeftLow + "   " + valMidLow + "   " + valRightLow);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.addData("PoseX", mecanumDrive.getPoseEstimate().getX());
            telemetry.addData("PoseY", mecanumDrive.getPoseEstimate().getY());
             */
            telemetry.addData("start x ", mecanumDrive.getPoseEstimate().getX());
            telemetry.addData("start y ", mecanumDrive.getPoseEstimate().getY());
            telemetry.addData("start heading ", mecanumDrive.getPoseEstimate().getHeading());

            telemetry.update();

            //AutoUtil.takeCube(this);

            int chosen = 0;

            if (valLeftLow > 0) chosen = 1;
            if (valMidLow > 0) chosen = 2;
            if (valRightLow > 0) chosen = 3;

            //fortat
            chosen = 1;

            switch(chosen){
                case 1:
                    new Case1(this).runAuto();
                    break;

            }

        }
    }


    private void initialize(){
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        robot = new RobotDefinition(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClawOpen(robot.getExcavator(), true);
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

        private CameraAdjusting.StageSwitchingPipeline.Stage stageToRenderToViewport = CameraAdjusting.StageSwitchingPipeline.Stage.detection;
        private CameraAdjusting.StageSwitchingPipeline.Stage[] stages = CameraAdjusting.StageSwitchingPipeline.Stage.values();

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
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 120, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //get values from frame
            double[] pixMidLow = thresholdMat.get((int)(input.rows()* midPosLow[1]), (int)(input.cols()* midPosLow[0]));//gets value at circle
            valMidLow = (int)pixMidLow[0];

            double[] pixLeftLow = thresholdMat.get((int)(input.rows()* leftPosLow[1]), (int)(input.cols()* leftPosLow[0]));//gets value at circle
            valLeftLow = (int)pixLeftLow[0];

            double[] pixRightLow = thresholdMat.get((int)(input.rows()* rightPosLow[1]), (int)(input.cols()* rightPosLow[0]));//gets value at circle
            valRightLow = (int)pixRightLow[0];

            //create three points
            Point pointMidLow = new Point((int)(input.cols()* midPosLow[0]), (int)(input.rows()* midPosLow[1]));
            Point pointLeftLow = new Point((int)(input.cols()* leftPosLow[0]), (int)(input.rows()* leftPosLow[1]));
            Point pointRightLow = new Point((int)(input.cols()* rightPosLow[0]), (int)(input.rows()* rightPosLow[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMidLow,5, new Scalar( 255, 255, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeftLow,5, new Scalar( 255, 255, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRightLow,5, new Scalar( 255, 255, 0 ),1 );//draws ci

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 4);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 4);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 4);

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPosLow[0]-rectWidth/2),
                            input.rows()*(leftPosLow[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPosLow[0]+rectWidth/2),
                            input.rows()*(leftPosLow[1]+rectHeight/2)),
                    new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPosLow[0]-rectWidth/2),
                            input.rows()*(midPosLow[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPosLow[0]+rectWidth/2),
                            input.rows()*(midPosLow[1]+rectHeight/2)),
                    new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPosLow[0]-rectWidth/2),
                            input.rows()*(rightPosLow[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPosLow[0]+rectWidth/2),
                            input.rows()*(rightPosLow[1]+rectHeight/2)),
                    new Scalar(255, 0, 0), 2);

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

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }

    public SampleMecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public ElapsedTime getRuntimeElapsed() {
        return runtime;
    }

    public static int getValLeftLow() {
        return valLeftLow;
    }

    public RobotDefinition getRobot() {
        return robot;
    }

    public static int getValMidLow() {
        return valMidLow;
    }

}