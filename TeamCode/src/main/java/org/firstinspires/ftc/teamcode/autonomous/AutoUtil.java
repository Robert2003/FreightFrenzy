package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutoUtil {

    public static void setClawOpen(Servo servo, boolean open){
        double pos = (open ? 0 : .15);
        servo.setPosition(pos);
    }

    public static void armToPosition(DcMotorEx armMotor, int position){
        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    public static void plateToPosition(DcMotorEx plateMotor, int position){
        plateMotor.setTargetPosition(position);
        plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        plateMotor.setPower(1);
    }

    public static void alignWithCube(SampleMecanumDrive drive, int left, int mid, int right, Servo servo){
        double move[] = {0,0.1, 2,10};

        int pos = 3;
        if(left > 0)
            pos = 1;
        if(right > 0)
            pos = 2;
        if(mid > 0)
            pos = 3;

        TrajectorySequence trajSeq;

        trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(move[pos])
                .waitSeconds(2)
                .forward(20)
                .waitSeconds(2)
                .addTemporalMarker(() -> setClawOpen(servo, false))
                .back(20)
                .addTemporalMarker(() -> setClawOpen(servo, true))
                .strafeLeft(move[pos])
                .waitSeconds(1.5)
                .build();
        drive.followTrajectorySequence(trajSeq);
    }

    /*
    public static void alignWithCube(SampleMecanumDrive drive, int left, Servo servo, double distance){
        double moveInterval = 1;
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(moveInterval)
                .build();
        if(left == 0 && distance < 20){
            drive.followTrajectory(traj);
            distanceMoved += moveInterval;
        }
        if(distanceMoved == 20) {
            distanceMoved /= 2;
            Trajectory defTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(distanceMoved)
                    .build();
            drive.followTrajectory(defTraj);
        }
       TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(5)
                .waitSeconds(2)
                .addTemporalMarker(() -> setClawOpen(servo, false))
                .back(5)
                .waitSeconds(0.5)
                .strafeLeft(distanceMoved)
                .waitSeconds(1.5)
                .build();
        drive.followTrajectorySequence(trajSeq);
    }
     */

}
