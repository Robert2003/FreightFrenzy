package org.firstinspires.ftc.teamcode.autonomous.cases;

import android.view.autofill.AutofillId;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.detection.FrenzyDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Case1 {

    FrenzyDetection auto;

    Trajectory shippingTraj1, frontWareTraj1, enterWareTraj1;
    TrajectorySequence enterTraj;

    double startX;

    public Case1(FrenzyDetection auto){
        this.auto = auto;
        shippingTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(15, 31)) // x era 14.75
                .build();
        /*
        enterTraj = auto.getMecanumDrive().trajectorySequenceBuilder(shippingTraj1.end())
                .back(5)
                .turn(Math.toRadians(-90))
                .strafeRight(10)
                .forward(40)
                .build();
         */

        frontWareTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj1.end())
                .lineToLinearHeading(new Pose2d(-13, 10, Math.toRadians(-145)))
                .build(); //new Pose2d(-3, 10)


        enterWareTraj1 = auto.getMecanumDrive().trajectoryBuilder(frontWareTraj1.end())
                .forward(25)
                .addTemporalMarker(0.2,
                        () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 20))
                .build();
    }

    public void runAuto(){
        AutoUtil.setClawOpen(auto.getRobot().getExcavator(), false);
        auto.sleep(100);
        AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 600);
        //while(auto.getRobot().getArmMotor().isBusy());
        auto.sleep(1000);
        auto.getMecanumDrive().followTrajectory(shippingTraj1);
        auto.sleep(1000);
        AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
        auto.sleep(400);
        auto.getMecanumDrive().followTrajectory(frontWareTraj1);
        auto.getMecanumDrive().followTrajectory(enterWareTraj1);
        auto.sleep(500);
        AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 20);
        auto.getRuntimeElapsed().reset();
        auto.getMecanumDrive().setPoseEstimate(new Pose2d(0, 0, 0));
        startX = auto.getMecanumDrive().getPoseEstimate().getX();
        while(auto.opModeIsActive()) {
            AutoUtil.takeCube(auto, startX);
        }
        auto.telemetry.addData("rotatie ", Math.toDegrees(
                auto.getMecanumDrive().getPoseEstimate().getHeading()) + " timp "
        + auto.getRuntimeElapsed());
        auto.telemetry.update();
    }

}
