package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.detection.FrenzyDetection;
import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
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

    public static void takeCube(FrenzyDetection auto, double startX){
        SampleMecanumDrive mecanumDrive = auto.getMecanumDrive();
        int valMidLow = auto.getValMidLow(); //am pus mid in loc de left
        ElapsedTime runtime = auto.getRuntimeElapsed();
        RobotDefinition robot = auto.getRobot();

        /*
        double distance = 0;
        double maxDistance = 21;
        double moveInterval = 1.5;
         */
        double moveSpeed = .2;
        double maxTime = 2;

        int status = -1;

        auto.telemetry.addData("Info", "ia cuburi\n-----------");
        auto.telemetry.addData("startX", startX);
        auto.telemetry.addData("poz x", mecanumDrive.getPoseEstimate().getX());
        auto.telemetry.addData("poz y", mecanumDrive.getPoseEstimate().getY());
        auto.telemetry.addData("status", status);
        auto.telemetry.update();

        if (valMidLow == 0 && runtime.seconds() <= maxTime) {
            mecanumDrive.setMotorPowers(-moveSpeed, moveSpeed, -moveSpeed, moveSpeed);
            status = 1;
        } else {
            status = 2;
            if(startX == mecanumDrive.getPoseEstimate().getX())
                startX += 0.1;
            mecanumDrive.setMotorPowers(0, 0, 0, 0);
            if (runtime.seconds() > maxTime) {
                Trajectory defTraj = mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                        .lineTo(new Vector2d(startX, mecanumDrive.getPoseEstimate().getY()))
                        .build();
                mecanumDrive.followTrajectory(defTraj);
            }
            runtime.reset();

            mecanumDrive.followTrajectory(mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .forward(14).build());
            mecanumDrive.followTrajectory(mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .back(0.5).build());
            AutoUtil.setClawOpen(robot.getExcavator(), false);
            auto.sleep(300);
            mecanumDrive.followTrajectory(mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .back(13.5).build());
            AutoUtil.armToPosition(robot.getArmMotor(), 1825);

            mecanumDrive.followTrajectory(mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-1, mecanumDrive.getPoseEstimate().getY()))
                    .build());
            mecanumDrive.followTrajectory(mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .back(25).build());
            AutoUtil.plateToPosition(robot.getPlateMotor(), -1305);
            mecanumDrive.followTrajectory(mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(14.75, 31)) // era -50;-15
                    .build());
            AutoUtil.setClawOpen(robot.getExcavator(), true);
            auto.sleep(300);
            mecanumDrive.followTrajectory(mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .strafeRight(15).build());
            AutoUtil.plateToPosition(robot.getPlateMotor(), 0);
            auto.sleep(100);
            AutoUtil.armToPosition(robot.getArmMotor(), 20);
            mecanumDrive.followTrajectory(mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .forward(50).build());
            runtime.reset();
        }
        mecanumDrive.update();
    }


}
