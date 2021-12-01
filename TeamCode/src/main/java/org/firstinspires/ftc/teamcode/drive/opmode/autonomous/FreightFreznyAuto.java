package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.cases.Case1;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class FreightFreznyAuto extends LinearOpMode {

    SampleMecanumDrive mecanumDrive;
    RobotDefinition robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        new Case1(this).runAuto();
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
