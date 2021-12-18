package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.cases.Case1;
import org.firstinspires.ftc.teamcode.autonomous.detection.StageSwitchingPipeline;
import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "FreightFrenzyAuto")
public class FreightFrenzyAuto extends LinearOpMode {

    OpenCvCamera phoneCam;
    StageSwitchingPipeline pipeline;
    private final int rows = 640;
    private final int cols = 480;

    SampleMecanumDrive mecanumDrive;
    RobotDefinition robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            runAuto();
        }
    }

    private void initialize() {

        //CAMERA

        pipeline = new StageSwitchingPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(pipeline);//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Values", pipeline.valLeft + "   " + pipeline.valMid + "   " +
                pipeline.valRight);
        telemetry.addData("Height", rows);
        telemetry.addData("Width", cols);
        telemetry.update();

        //ROBOT

        robot = new RobotDefinition(hardwareMap);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClawOpen(robot.getExcavator(), false);



    }

    private void runAuto() {
        int runCase = 1;
        if (pipeline.valMid != 0)
            runCase = 2;
        else if (pipeline.valRight != 0)
            runCase = 3;
        switch (runCase) {
            case 1:
                new Case1(this).runAuto();
        }
    }

    public RobotDefinition getRobot() {
        return robot;
    }

    public SampleMecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

}
