package org.firstinspires.ftc.teamcode.autonomous.national.inter;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.national.inter.option.ForcedCase;
import org.firstinspires.ftc.teamcode.autonomous.national.inter.option.Side;
import org.firstinspires.ftc.teamcode.autonomous.national.inter.option.TeamCompatible;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class SelectionCaseRedBag {

    CameraAdjustingSyncedRedBag auto;
    Side side;
    ForcedCase forcedCase;
    TeamCompatible teamCompatible;

    Trajectory alignWithWall3, goToShippingHub2, deliverPreloadBox,goToCarousel,storageUnitPark,alignWithWall,enterWarehouse,goToShippingHub,alignWithWall2,strafeToPark,park,goToShippingHub4,alignWithWall5,goToShippingHub3,alignWithWall4;
    TrajectorySequence parkingSoft,goToPark,exitWarehouse2,enterWarehouse4,exitWarehouse4,enterWarehouse3,goToWare,getCube,exitWarehouse,collectDuck,alignWithCarousel,deliverDuck,enterWarehouse2,exitWarehouse3;

    int armGoTo;
    int sign;

    public SelectionCaseRedBag(CameraAdjustingSyncedRedBag auto, int armGoTo) {
        this.auto = auto;
        this.armGoTo = armGoTo;
        this.side = Side.RED;
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
                    .trajectorySequenceBuilder(enterWarehouse.end())
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
                    .lineToLinearHeading(new Pose2d(-7,sign * -2, Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.22, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1100))
                    .addTemporalMarker(1.8, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .build();
            enterWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall.end())
                    .forward(24)
                    .forward(12,
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            exitWarehouse = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(enterWarehouse.end())
                    .lineToLinearHeading(new Pose2d(-4, sign * -7, Math.toRadians(sign * -90)))
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1775))
                    .build();
            goToShippingHub = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse.end())
                    .lineToLinearHeading(new Pose2d(12,sign * 18, Math.toRadians(sign * (5))))
                    .build();
            alignWithWall2 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub.end())
                    .lineToLinearHeading(new Pose2d(-7,sign * 10, Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            enterWarehouse2 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall2.end())
                    .forward(19)
                    .turn(Math.toRadians(sign * (10)))
                    .forward(8)
                    .forward(15, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            exitWarehouse2 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(enterWarehouse.end())
                    .lineToLinearHeading(new Pose2d(-10, sign * -17, Math.toRadians(sign * -90)))
                    .back(17)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1775))
                    .build();
            goToShippingHub2 = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse2.end())
                    .lineToLinearHeading(new Pose2d(9,sign * 22, Math.toRadians(sign * (10)))) // era 34
                    .build();
            alignWithWall3 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub2.end())
                    .lineToLinearHeading(new Pose2d(-15,sign * 7,Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            enterWarehouse3 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall3.end())
                    .forward(23)
                    .turn(Math.toRadians(sign * (25)))
                    .forward(12)
                    .forward(8,
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            exitWarehouse3 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(enterWarehouse.end())
                    .lineToLinearHeading(new Pose2d(-25, sign * -9, Math.toRadians(sign * -90)))
                    .back(15)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1775))
                    .build();
            goToShippingHub3 = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse3.end())
                    .lineToLinearHeading(new Pose2d(0,sign * 38, Math.toRadians(sign * (5)))) // era 34
                    .build();
            alignWithWall4 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub3.end())
                    .lineToLinearHeading(new Pose2d(-30,sign * 4,Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            enterWarehouse4 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall4.end())
                    .forward(27)
                    .turn(Math.toRadians(sign * (25)))
                    .forward(10)
                    .forward(10,
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            exitWarehouse4 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(enterWarehouse4.end())
                    .back(2,SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .back(50)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1900))
                    .build();
            goToShippingHub4 = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse4.end())
                    .lineToLinearHeading(new Pose2d(-3,sign * 20, Math.toRadians(sign * (5)))) // era 34
                    .build();
            alignWithWall5 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub4.end())
                    .lineToLinearHeading(new Pose2d(-21,sign * 4,Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            parkingSoft = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall4.end())
                    .strafeRight(sign * 4)
                    .forward(32)
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
            auto.getMecanumDrive().followTrajectorySequence(exitWarehouse);
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
            auto.sleep(200);
            auto.getMecanumDrive().followTrajectory(alignWithWall);
            //auto.sleep(50);
            auto.getMecanumDrive().followTrajectory(enterWarehouse);
            //auto.sleep(100);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), false);
            auto.sleep(50);
            auto.getMecanumDrive().followTrajectorySequence(exitWarehouse);
            auto.getMecanumDrive().followTrajectory(goToShippingHub);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(200);
            auto.getMecanumDrive().followTrajectory(alignWithWall2);
            auto.getMecanumDrive().followTrajectorySequence(enterWarehouse2);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), false);
            auto.sleep(50);
            auto.getMecanumDrive().followTrajectorySequence(exitWarehouse2);
            auto.getMecanumDrive().followTrajectory(goToShippingHub2);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(200);
            auto.getMecanumDrive().followTrajectory(alignWithWall3);
            auto.getMecanumDrive().followTrajectorySequence(enterWarehouse3);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), false);
            auto.sleep(50);
            auto.getMecanumDrive().followTrajectorySequence(exitWarehouse3);
            auto.getMecanumDrive().followTrajectory(goToShippingHub3);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(200);
            auto.getMecanumDrive().followTrajectory(alignWithWall4);
            auto.getMecanumDrive().followTrajectorySequence(enterWarehouse4);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), false);
            auto.sleep(50);
            auto.getMecanumDrive().followTrajectorySequence(exitWarehouse4);
            auto.getMecanumDrive().followTrajectory(goToShippingHub4);
            AutoUtil.setClawOpen(auto.getRobot(), auto.getRobot().getExcavator(), true);
            auto.sleep(200);
            auto.getMecanumDrive().followTrajectory(alignWithWall5);
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