package org.firstinspires.ftc.teamcode.autonomous.national;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.detection.FrenzyDetection;
import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


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

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        FrenzyCamera frenzyCamera = new FrenzyCamera();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new FrenzyCamera.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(frenzyCamera.getRows(), frenzyCamera.getCols(), OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        //func.init(hardwareMap);

        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Values (Cap)", frenzyCamera.getValLeft() + "   " + frenzyCamera.getValMid() + "   " + frenzyCamera.getValRight());
            telemetry.addData("Value (Mobile)", frenzyCamera.getValMobile());
            telemetry.addData("Rows", frenzyCamera.getRows());
            telemetry.addData("Cols", frenzyCamera.getCols());

            telemetry.update();

        }
    }

}