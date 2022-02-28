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
@Autonomous(name = "Camera Adjusting Synced Old", group = "Ajustare Joc")
//@Disabled//comment out this line before using
public class CameraAdjustingSyncedOld extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

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

            telemetry.addData("Values (Cap)", frenzyDet.getValLeft() + "   " + frenzyDet.getValMid() + "   " + frenzyDet.getValRight());
            telemetry.addData("LValues (Cargo)", frenzyDet.getValLeftLow() + "   " + frenzyDet.getValMidLow() + "   " + frenzyDet.getValRightLow());
            telemetry.addData("Rows", frenzyDet.getRows());
            telemetry.addData("Cols", frenzyDet.getCols());

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

    public RobotDefinition getRobot() {
        return robot;
    }

}