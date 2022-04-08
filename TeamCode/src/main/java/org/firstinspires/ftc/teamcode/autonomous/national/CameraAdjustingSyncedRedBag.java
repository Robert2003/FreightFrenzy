package org.firstinspires.ftc.teamcode.autonomous.national;

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
@Autonomous(name = "Camera Adjusting Synced Red Bag", group = "Ajustare Joc")
//@Disabled//comment out this line before using
public class CameraAdjustingSyncedRedBag extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera phoneCam;

    RobotDefinition robot;
    SampleMecanumDrive mecanumDrive;

    public SampleMecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

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

        initialize();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //initialize();
            telemetry.addData("Values (Cap)", frenzyCamera.getValLeft() + "   " + frenzyCamera.getValMid() + "   " + frenzyCamera.getValRight());
            telemetry.addData("Value (Mobile)", frenzyCamera.getValMobile());
            telemetry.addData("Rows", frenzyCamera.getRows());
            telemetry.addData("Cols", frenzyCamera.getCols());
            int armGoTo = 1125;
            if (frenzyCamera.getValRight() != 0)
                armGoTo = 1775;
            else if (frenzyCamera.getValLeft() != 0)
                armGoTo = 650;
            new SelectionCaseRedBag(this, armGoTo).runAuto();
            telemetry.update();
            sleep(30000);

        }

    }

    private void initialize(){
        robot = new RobotDefinition(hardwareMap);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClawOpen(robot, robot.getExcavator(), false);
    }

    public RobotDefinition getRobot() {
        return robot;
    }
}