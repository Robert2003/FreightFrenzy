package org.firstinspires.ftc.teamcode.autonomous.cases.noautocube;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.detection.FrenzyDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Case1NoCube {

    FrenzyDetection auto;

    Trajectory shippingTraj1, duckTraj1, duckTraj2, parkTraj, backTraj, shippingTraj2, parkTraj2, strafeTraj;

    double startY;

    public Case1NoCube(FrenzyDetection auto) {
        this.auto = auto;
        shippingTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(15, -31)) // x era 14.75
                .build();
        duckTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj1.end())
                .lineToLinearHeading(new Pose2d(5,20,Math.toRadians(-90)))
                .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f))
                .build();
        duckTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(duckTraj1.end())
                .back(8)
                .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f))
                .build();
        parkTraj = auto.getMecanumDrive()
                .trajectoryBuilder(duckTraj2.end())
                .lineToLinearHeading(new Pose2d(4,-80, Math.toRadians(-90)))
                .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 35))
                .build();
        backTraj = auto.getMecanumDrive()
                .trajectoryBuilder(parkTraj.end())
                .back(40)
                .build();
        shippingTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(backTraj.end())
                .lineTo(new Vector2d(35, -20)) // x era 14.75
                .build();
        strafeTraj = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj2.end())
                .strafeRight(30)
                .addTemporalMarker(.1, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(), 0))
                .build();
        parkTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(strafeTraj.end())
                .lineToLinearHeading(new Pose2d(4,-75, Math.toRadians(-90)))
                .addTemporalMarker(.4, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 35))
                .build();
    }

    public void runAuto() {
        AutoUtil.setClawOpen(auto.getRobot().getExcavator(), false);
        auto.telemetry.addData("x ", auto.getMecanumDrive().getPoseEstimate().getX());
        auto.telemetry.addData("y ", auto.getMecanumDrive().getPoseEstimate().getY());
        auto.telemetry.update();
        auto.sleep(200);
        AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 600);
        //while(auto.getRobot().getArmMotor().isBusy());
        auto.sleep(1000);
        auto.getMecanumDrive().followTrajectory(shippingTraj1);
        auto.telemetry.addData("x ", auto.getMecanumDrive().getPoseEstimate().getX());
        auto.telemetry.addData("y ", auto.getMecanumDrive().getPoseEstimate().getY());
        auto.telemetry.update();
        //auto.sleep(1000);
        AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
        //auto.sleep(400);
        auto.getMecanumDrive().followTrajectory(duckTraj1);
        //auto.sleep(500);
        AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 600);
        auto.getMecanumDrive().followTrajectory(duckTraj2);
        auto.sleep(3000);
        AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), 0);
        auto.telemetry.addData("x ", auto.getMecanumDrive().getPoseEstimate().getX());
        auto.telemetry.addData("y ", auto.getMecanumDrive().getPoseEstimate().getY());
        auto.telemetry.update();
        auto.getMecanumDrive().followTrajectory(parkTraj);
        AutoUtil.setClawOpen(auto.getRobot().getExcavator(), false);
        auto.sleep(100);
        auto.getMecanumDrive().followTrajectory(backTraj);
        AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 1825);
        AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(), -1305);
        auto.getMecanumDrive().followTrajectory(shippingTraj2);
        AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
        auto.sleep(300);
        auto.getMecanumDrive().followTrajectory(strafeTraj);
        auto.getMecanumDrive().followTrajectory(parkTraj2);
        auto.sleep(10000);
    }

}
