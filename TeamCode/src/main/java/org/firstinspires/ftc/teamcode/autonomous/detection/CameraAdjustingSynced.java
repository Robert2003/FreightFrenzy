package org.firstinspires.ftc.teamcode.autonomous.detection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
 * <p>
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 * <p>
 * monitor: 640 x 480
 * YES
 */
@Autonomous(name = "Camera Adjusting Synced", group = "Ajustare Joc")
//@Disabled//comment out this line before using
public class CameraAdjustingSynced extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = 0;
    private static int valLeft = 0;
    private static int valRight = 0;
    private static int valMidLow = 0;
    private static int valLeftLow = 0;
    private static int valRightLow = 0;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.4f / 8f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    public static double[] midPos = {4 / 8.0 + offsetX, 2.3 / 8.0 + offsetY};//0 = col, 1 = row
    public static double[] leftPos = {1 / 8.0 + offsetX, 2.3 / 8.0 + offsetY}; // era 2
    public static double[] rightPos = {6.3 / 8.0 + offsetX, 2.3 / 8.0 + offsetY}; // era 6
    //moves all rectangles right or left by amount. units are in ratio to monitor
    public static double[] midPosLow = {4 / 8.0 + offsetX, 6 / 8.0 + offsetY};//0 = col, 1 = row
    public static double[] leftPosLow = {0.4 / 8.0 + offsetX, 6 / 8.0 + offsetY}; // era 2
    public static double[] rightPosLow = {7.6 / 8.0 + offsetX, 6 / 8.0 + offsetY}; // era 6

    private final int rows = 1280; //640 x 480
    private final int cols = 720;

    OpenCvCamera phoneCam;

    RobotDefinition robot;
    SampleMecanumDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        FrenzyDetection frenzyDet = new FrenzyDetection();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new FrenzyDetection.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(frenzyDet.getRows(), frenzyDet.getCols(), OpenCvCameraRotation.UPRIGHT);//display on RC
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

            telemetry.addData("Values (Cap)", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("LValues (Cargo)", valLeftLow + "   " + valMidLow + "   " + valRightLow);
            telemetry.addData("Rows", rows);
            telemetry.addData("Cols", cols);

            telemetry.update();

        }
    }


    private void initialize() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        robot = new RobotDefinition(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClawOpen(robot.getExcavator(), true);
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