package org.firstinspires.ftc.teamcode.autonomous.cases.noautocube;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.detection.FrenzyDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class UniversalCase3 {

    FrenzyDetection auto;

    Trajectory shippingTraj1, shippingTraj2, wareTraj1, enterTraj1, backTraj1, wareTraj2, backTraj2, enterTraj2, shippingTraj3;
    Trajectory shippingTraj4, wareTraj3, enterTraj3, backTraj3, wareTraj4, enterTraj4;

    Servo claw;
    DcMotorEx flyWheel, armMotor, plateMotor;

    int armGoTo;

    public UniversalCase3(FrenzyDetection auto, int armGoTo) {
        this.auto = auto;
        this.armGoTo = armGoTo;
        claw = auto.getRobot().getExcavator();
        flyWheel = auto.getRobot().getFlyWheel();
        armMotor = auto.getRobot().getArmMotor();
        plateMotor = auto.getRobot().getPlateMotor();
        shippingTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(14, -31)) // x era 14.75
                .build();
        wareTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj1.end())
                .lineToLinearHeading(new Pose2d(-24, -62, Math.toRadians(-120)))
                .build();
        enterTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(wareTraj1.end())
                .forward(23)
                .build();
        backTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(enterTraj1.end())
                .back(28)
                .build();
        shippingTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(backTraj1.end())
                .lineTo(new Vector2d(15, -53)) // x era 14.75
                .build();
        wareTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj2.end())
                .lineToLinearHeading(new Pose2d(-24, -62, Math.toRadians(-120)))
                .build();
        enterTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(wareTraj2.end())
                .forward(29)
                .build();
        backTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(enterTraj2.end())
                .back(32)
                .build();
        shippingTraj3 = auto.getMecanumDrive()
                .trajectoryBuilder(backTraj2.end())
                .lineTo(new Vector2d(15, -53)) // x era 14.75
                .build();
        wareTraj3 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj3.end())
                .lineToLinearHeading(new Pose2d(-24, -62, Math.toRadians(-120)))
                .build();
        enterTraj3 = auto.getMecanumDrive()
                .trajectoryBuilder(wareTraj3.end())
                .forward(30)
                .build();
        backTraj3 = auto.getMecanumDrive()
                .trajectoryBuilder(enterTraj3.end())
                .back(35)
                .build();
        shippingTraj4 = auto.getMecanumDrive()
                .trajectoryBuilder(backTraj3.end())
                .lineTo(new Vector2d(15, -53)) // x era 14.75
                .build();
        wareTraj4 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj4.end())
                .lineToLinearHeading(new Pose2d(-24, -62, Math.toRadians(-120)))
                .build();
        enterTraj4 = auto.getMecanumDrive()
                .trajectoryBuilder(wareTraj4.end())
                .forward(24)
                .build();
    }

    public void runAuto() {
        AutoUtil.setClawOpen(claw, false);
        auto.sleep(350);
        AutoUtil.armToPosition(armMotor, armGoTo);
        auto.getMecanumDrive().followTrajectory(shippingTraj1);
        AutoUtil.setClawOpen(claw, true);
        auto.getMecanumDrive().followTrajectory(wareTraj1);
        AutoUtil.plateToPosition(plateMotor, 0);
        AutoUtil.armToPosition(armMotor, auto.getRobot().getZeroArm());
        auto.getMecanumDrive().followTrajectory(enterTraj1);
        AutoUtil.setClawOpen(claw, false);
        auto.getMecanumDrive().followTrajectory(backTraj1);
        AutoUtil.armToPosition(armMotor, 1805);
        AutoUtil.plateToPosition(plateMotor, -1800);
        auto.getMecanumDrive().followTrajectory(shippingTraj2);
        AutoUtil.setClawOpen(claw, true);
        auto.sleep(300);

        auto.getMecanumDrive().followTrajectory(wareTraj2);
        AutoUtil.plateToPosition(plateMotor, 0);
        AutoUtil.armToPosition(armMotor, auto.getRobot().getZeroArm());
        auto.getMecanumDrive().followTrajectory(enterTraj2);
        AutoUtil.setClawOpen(claw, false);
        auto.getMecanumDrive().followTrajectory(backTraj2);
        AutoUtil.armToPosition(armMotor, 1805);
        AutoUtil.plateToPosition(plateMotor, -1800);
        auto.getMecanumDrive().followTrajectory(shippingTraj3);
        AutoUtil.setClawOpen(claw, true);
        auto.sleep(300);

        auto.getMecanumDrive().followTrajectory(wareTraj3);
        AutoUtil.plateToPosition(plateMotor, 0);
        AutoUtil.armToPosition(armMotor, auto.getRobot().getZeroArm());
        auto.getMecanumDrive().followTrajectory(enterTraj3);
        AutoUtil.setClawOpen(claw, false);
        auto.getMecanumDrive().followTrajectory(backTraj3);
        AutoUtil.armToPosition(armMotor, 1805);
        AutoUtil.plateToPosition(plateMotor, -1505);
        auto.getMecanumDrive().followTrajectory(shippingTraj4);
        AutoUtil.setClawOpen(claw, true);
        auto.sleep(300);

        AutoUtil.plateToPosition(plateMotor, 0);
        auto.getMecanumDrive().followTrajectory(wareTraj4);
        AutoUtil.armToPosition(armMotor, auto.getRobot().getZeroArm());
        auto.getMecanumDrive().followTrajectory(enterTraj4);

        auto.sleep(10000);
    }

}
