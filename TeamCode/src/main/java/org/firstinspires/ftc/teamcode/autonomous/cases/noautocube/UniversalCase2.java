package org.firstinspires.ftc.teamcode.autonomous.cases.noautocube;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.detection.FrenzyDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class UniversalCase2 {

    FrenzyDetection auto;
    TrajectorySequence autoTraj;
    Servo claw;
    DcMotorEx flyWheel, armMotor;

    int armGoTo;

    public UniversalCase2(FrenzyDetection auto, int armGoTo) {
        this.auto = auto;
        this.armGoTo = armGoTo;
        claw = auto.getRobot().getExcavator();
        flyWheel = auto.getRobot().getFlyWheel();
        armMotor = auto.getRobot().getArmMotor();
        autoTraj = auto.getMecanumDrive().trajectorySequenceBuilder(new Pose2d())
                .addDisplacementMarker(() -> AutoUtil.setClawOpen(claw, false))
                .waitSeconds(.25)
                .lineTo(new Vector2d(15, -31))
                .addDisplacementMarker(() -> AutoUtil.setClawOpen(claw, true))
                .lineToLinearHeading(new Pose2d(5,20, Math.toRadians(-90)))
                .addTemporalMarker(.3, () -> {
                    AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.7f);
                    AutoUtil.armToPosition(armMotor, 600);
                    auto.telemetry.addData("arm la", armMotor.getCurrentPosition());
                    auto.telemetry.update();
                })
                .back(8)
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(4,-80, Math.toRadians(-90)))
                .addTemporalMarker(.1, () -> {
                    AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), 0);
                    AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 15);
                })
                .addDisplacementMarker(() -> AutoUtil.setClawOpen(claw, false))
                .back(40)
                .lineTo(new Vector2d(31, -20))
                .addDisplacementMarker(() -> AutoUtil.setClawOpen(claw, true))
                .strafeRight(30)
                .addTemporalMarker(.1, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(), 0))
                .addTemporalMarker(.4, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 15))
                .lineToLinearHeading(new Pose2d(7,-75, Math.toRadians(-90)))
                .build();
    }

    public void runAuto() {
        auto.getMecanumDrive().followTrajectorySequence(autoTraj);
    }

}
