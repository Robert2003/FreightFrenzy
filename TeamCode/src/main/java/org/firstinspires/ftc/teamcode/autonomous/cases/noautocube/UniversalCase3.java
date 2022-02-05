package org.firstinspires.ftc.teamcode.autonomous.cases.noautocube;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.detection.FrenzyDetection;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class UniversalCase3 {

    FrenzyDetection auto;

    Trajectory shippingTraj1, shippingTraj2, wareTraj1, enterTraj1, backTraj1, wareTraj2, backTraj2, enterTraj2, shippingTraj3;
    Trajectory shippingTraj4, wareTraj3, enterTraj3, backTraj3, wareTraj4, enterTraj4;
    TrajectorySequence turnTraj1;

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
                .lineTo(new Vector2d(20.5, -32),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // x era 14.75
                .build();
        wareTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj1.end())
                .addTemporalMarker(.6, () -> AutoUtil.armToPosition(armMotor, auto.getRobot().getZeroArm()))
                .lineToLinearHeading(new Pose2d(-24, -62, Math.toRadians(-150)),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        enterTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(wareTraj1.end())
                .forward(20)
                .build();
        backTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(enterTraj1.end())
                .back(35,
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        shippingTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(backTraj1.end())
                .lineTo(new Vector2d(17, -77),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // x era 14.75
                .build();
        wareTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj2.end())
                .lineToLinearHeading(new Pose2d(-24, -56, Math.toRadians(-120)),
                        SampleMecanumDrive.getVelocityConstraint(48, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        enterTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(wareTraj2.end())
                .forward(33.5)
                .build();
        backTraj2 = auto.getMecanumDrive()
                .trajectoryBuilder(enterTraj2.end())
                .back(32)
                .build();
        shippingTraj3 = auto.getMecanumDrive()
                .trajectoryBuilder(backTraj2.end())
                .lineTo(new Vector2d(16, -56),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // x era 14.75
                .build();
        wareTraj3 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj3.end())
                .lineToLinearHeading(new Pose2d(-24, -56, Math.toRadians(-120)))
                .build();
        turnTraj1 = auto.getMecanumDrive()
                .trajectorySequenceBuilder(wareTraj3.end())
                .turn(Math.toRadians(30))
                .build();
        enterTraj3 = auto.getMecanumDrive()
                .trajectoryBuilder(turnTraj1.end())
                .forward(31)
                .build();
        backTraj3 = auto.getMecanumDrive()
                .trajectoryBuilder(enterTraj3.end())
                .back(40, SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        shippingTraj4 = auto.getMecanumDrive()
                .trajectoryBuilder(backTraj3.end())
                .lineTo(new Vector2d(10, -36.5)) // x era 14.75
                .build();
        wareTraj4 = auto.getMecanumDrive()
                .trajectoryBuilder(shippingTraj4.end())
                .lineToLinearHeading(new Pose2d(-24, -55, Math.toRadians(-120)),
                        SampleMecanumDrive.getVelocityConstraint(37, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.3, () -> AutoUtil.armToPosition(armMotor, 350))
                .build();
        enterTraj4 = auto.getMecanumDrive()
                .trajectoryBuilder(wareTraj4.end())
                .forward(26, SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }

    public void runAuto() {
        AutoUtil.setClawOpen(claw, false);
        auto.sleep(350);
        AutoUtil.armToPosition(armMotor, armGoTo);
        auto.getMecanumDrive().followTrajectory(shippingTraj1);
        AutoUtil.setClawOpen(claw, true);
        AutoUtil.plateToPosition(plateMotor, 0);
        auto.getMecanumDrive().followTrajectory(wareTraj1);
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
        AutoUtil.plateToPosition(plateMotor, -1700);
        auto.getMecanumDrive().followTrajectory(shippingTraj3);
        AutoUtil.setClawOpen(claw, true);
        auto.sleep(300);

        auto.getMecanumDrive().followTrajectory(wareTraj3);
        AutoUtil.plateToPosition(plateMotor, 0);
        AutoUtil.armToPosition(armMotor, auto.getRobot().getZeroArm());
        auto.getMecanumDrive().followTrajectorySequence(turnTraj1);
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