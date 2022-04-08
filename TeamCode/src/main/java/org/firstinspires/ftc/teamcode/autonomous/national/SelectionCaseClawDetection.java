package org.firstinspires.ftc.teamcode.autonomous.national;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.national.option.ForcedCase;
import org.firstinspires.ftc.teamcode.autonomous.national.option.Side;
import org.firstinspires.ftc.teamcode.autonomous.national.option.TeamCompatible;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class SelectionCaseClawDetection {

    CameraAdjustingSyncedClawDetection auto;
    Side side;
    ForcedCase forcedCase;
    TeamCompatible teamCompatible;

    Trajectory alignWithWall3, goToShippingHub2, deliverPreloadBox,goToCarousel,storageUnitPark,alignWithWall,enterWarehouse,exitWarehouse,goToShippingHub,alignWithWall2,strafeToPark,park,goToShippingHub3,alignWithWall4;
    TrajectorySequence parkingSoft,goToPark,exitWarehouse2, enterWarehouse3,goToWare,getCube,collectDuck,alignWithCarousel,deliverDuck,enterWarehouse2,exitWarehouse3;

    int armGoTo;
    int sign;

    public SelectionCaseClawDetection(CameraAdjustingSyncedClawDetection auto, int armGoTo) {
        this.auto = auto;
        this.armGoTo = armGoTo;
        this.side = Side.BLUE;
        this.forcedCase = ForcedCase.DETECTION;
        this.teamCompatible = TeamCompatible.BAG_UN_CARRY;

        initializeTrajectories();
    }

    private void initializeTrajectories() {
        sign = 1;
        if(side == Side.BLUE) sign = -1;
        if(teamCompatible == TeamCompatible.SOFT_DELTA) {
            deliverPreloadBox = auto.getMecanumDrive()
                    .trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d( 18, sign * 20, Math.toRadians(sign * (35))))
                    .build();
            alignWithWall = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(-16,sign * 6, Math.toRadians(sign * (-130))))
                    .addTemporalMarker(.4226, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1100))
                    .addTemporalMarker(2, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .build();
            enterWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall.end())
                    .forward(31)
                    .build();
            exitWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(enterWarehouse.end())
                    .back(34)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1775))
                    .build();
            goToShippingHub = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse.end())
                    .lineToLinearHeading(new Pose2d(13,sign * 20, Math.toRadians(sign * (20))))
                    .build();
            alignWithWall2 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub.end())
                    .lineToLinearHeading(new Pose2d(-18,sign * 10,Math.toRadians(sign * (-130))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            parkingSoft = auto.getMecanumDrive().trajectorySequenceBuilder(alignWithWall2.end())
                    .forward(23)
                    .strafeLeft(sign * 30)
                    .forward(16)
                    .turn(Math.toRadians(sign * -140))
                    .strafeLeft(sign * 10)
                    .build();
        } else if(teamCompatible == TeamCompatible.BAG_UN_CARRY) {
            deliverPreloadBox = auto.getMecanumDrive()
                    .trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d( 18, sign * 23, Math.toRadians(sign * (10))))
                    .build();
            alignWithWall = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(-2,sign * -2, Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.37, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1100))
                    .addTemporalMarker(1.8, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .build();
            enterWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall.end())
                    .forward(34)
                    .build();
            exitWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(enterWarehouse.end())
                    .back(37)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1775))
                    .build();
            goToShippingHub = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse.end())
                    .lineToLinearHeading(new Pose2d(18,sign * 18, Math.toRadians(sign * (10))))
                    .build();
            alignWithWall2 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub.end())
                    .lineToLinearHeading(new Pose2d(-4,sign * 10, Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            enterWarehouse2 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall2.end())
                    .forward(21)
                    .turn(Math.toRadians(sign * (10)))
                    .forward(29)
                    .build();
            exitWarehouse2 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(enterWarehouse2.end())
                    .back(46)
                    //.strafeRight(sign * 8)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1825))
                    .build();
            goToShippingHub2 = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse2.end())
                    .lineToLinearHeading(new Pose2d(12,sign * 20, Math.toRadians(sign * (0)))) // era 34
                    .build();
            alignWithWall3 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub2.end())
                    .lineToLinearHeading(new Pose2d(-15,sign * 4,Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            enterWarehouse3 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall3.end())
                    .forward(23)
                    .turn(Math.toRadians(sign * (5)))
                    .forward(26)
                    .build();
            exitWarehouse3 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(enterWarehouse3.end())
                    .back(47)
                    //.strafeRight(sign * 8)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1825))
                    .build();
            goToShippingHub3 = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse3.end())
                    .lineToLinearHeading(new Pose2d(3,sign * 16, Math.toRadians(sign * (5)))) // era 34
                    .build();
            alignWithWall4 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub3.end())
                    .lineToLinearHeading(new Pose2d(-19,sign * 4,Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            parkingSoft = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall4.end())
                    .strafeRight(sign * 4)
                    .forward(38)
                    .strafeLeft(sign * 30)
                    .build();
        } else if(teamCompatible == TeamCompatible.NONE) {
            deliverPreloadBox = auto.getMecanumDrive()
                    .trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d( 18, sign * (-27)))
                    .build();
            goToCarousel = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(0,sign * 15,Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.5, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(), auto.getRobot().getZeroArm()))
                    .build();
            alignWithCarousel = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(goToCarousel.end())
                    .back(7,
                            SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f * sign))
                    .strafeRight(sign * 2.78)
                    .build();
            park = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithCarousel.end())
                    .lineToLinearHeading(new Pose2d(28, sign * 24, Math.toRadians(sign * (-90))))
                    .build();
        } else if(teamCompatible == TeamCompatible.IAU_UN_CARRY){
            deliverPreloadBox = auto.getMecanumDrive()
                    .trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d( 18, sign * 23, Math.toRadians(sign * (10))))
                    .build();
            alignWithWall = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(-3,sign * -2, Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.37, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1100))
                    .addTemporalMarker(1.8, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .build();
            enterWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall.end())
                    .forward(25)
                    .build();
            parkingSoft = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(enterWarehouse.end())
                    .strafeLeft(sign * 30)
                    .build();
        }

    }

    public void runAuto() {
        if(teamCompatible == TeamCompatible.SOFT_DELTA) {
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            //while(auto.getRobot().getArmMotor().isBusy());
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(alignWithWall);
            //auto.sleep(50);
            auto.getMecanumDrive().followTrajectory(enterWarehouse);
            //auto.sleep(100);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), false);
            auto.getMecanumDrive().followTrajectory(exitWarehouse);
            auto.getMecanumDrive().followTrajectory(goToShippingHub);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            //auto.sleep(50);
            auto.getMecanumDrive().followTrajectory(alignWithWall2);
            auto.getMecanumDrive().followTrajectorySequence(parkingSoft);
        } else if (teamCompatible == TeamCompatible.BAG_UN_CARRY) {
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            //while(auto.getRobot().getArmMotor().isBusy());
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(250);
            auto.getMecanumDrive().followTrajectory(alignWithWall);
            //auto.sleep(50);
            auto.getMecanumDrive().followTrajectory(enterWarehouse);
            //auto.sleep(100);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), false);
            auto.getMecanumDrive().followTrajectory(exitWarehouse);
            auto.getMecanumDrive().followTrajectory(goToShippingHub);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(250);
            auto.getMecanumDrive().followTrajectory(alignWithWall2);
            auto.getMecanumDrive().followTrajectorySequence(enterWarehouse2);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), false);
            auto.getMecanumDrive().followTrajectorySequence(exitWarehouse2);
            auto.getMecanumDrive().followTrajectory(goToShippingHub2);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(250);
            auto.getMecanumDrive().followTrajectory(alignWithWall3);
            auto.getMecanumDrive().followTrajectorySequence(enterWarehouse3);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), false);
            auto.getMecanumDrive().followTrajectorySequence(exitWarehouse3);
            auto.getMecanumDrive().followTrajectory(goToShippingHub3);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(250);
            auto.getMecanumDrive().followTrajectory(alignWithWall4);
            auto.getMecanumDrive().followTrajectorySequence(parkingSoft);
        } else if(teamCompatible == TeamCompatible.NONE) {
            auto.sleep(2000);
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            //while(auto.getRobot().getArmMotor().isBusy());
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(500);
            auto.getMecanumDrive().followTrajectory(goToCarousel);
            auto.getMecanumDrive().followTrajectorySequence(alignWithCarousel);
            auto.sleep(6000);
            auto.getMecanumDrive().followTrajectory(park);
            AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), 0);
        } else if(teamCompatible == TeamCompatible.IAU_UN_CARRY){
            auto.sleep(3000);
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(500);
            auto.getMecanumDrive().followTrajectory(alignWithWall);
            auto.getMecanumDrive().followTrajectory(enterWarehouse);
            auto.getMecanumDrive().followTrajectorySequence(parkingSoft);
        }
    }

}