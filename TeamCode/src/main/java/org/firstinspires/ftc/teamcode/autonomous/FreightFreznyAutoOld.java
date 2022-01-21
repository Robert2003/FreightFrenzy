package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class FreightFreznyAutoOld extends LinearOpMode {

    SampleMecanumDrive mecanumDrive;
    RobotDefinition robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        //new Case1(this).runAuto();
    }

    private void initialization(){
        robot = new RobotDefinition(hardwareMap);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClawOpen(robot.getExcavator(), false);
    }

    public RobotDefinition getRobot() {
        return robot;
    }

    public SampleMecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

}
