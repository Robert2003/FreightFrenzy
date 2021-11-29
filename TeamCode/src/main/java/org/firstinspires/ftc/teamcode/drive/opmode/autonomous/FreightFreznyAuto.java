package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class FreightFreznyAuto extends LinearOpMode {

    SampleMecanumDrive mecanumDrive;

    Servo servo = hardwareMap.get(Servo.class, "servo");

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        runAuto();
    }

    private void initialization(){
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClawOpen(servo, false);
    }

    private void runAuto(){
        TrajectorySequence trajSeq = mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate())
                .forward(11.5)
                .build();
        mecanumDrive.followTrajectorySequence(trajSeq);
        AutoUtil.setClawOpen(servo, true);
    }

}
