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

public class SelectionCase {

    FrenzySelection auto;
    Side side;
    ForcedCase forcedCase;
    TeamCompatible teamCompatible;

    Trajectory alignWithWall3, goToShippingHub2, enterWarehouse3, deliverPreloadBox,goToCarousel,storageUnitPark,alignWithWall,enterWarehouse,exitWarehouse,goToShippingHub,alignWithWall2,strafeToPark,park;
    TrajectorySequence parkingSoft,goToPark,exitWarehouse2,goToWare,getCube,collectDuck,alignWithCarousel,deliverDuck,enterWarehouse2;

    int armGoTo;
    int sign;

    public SelectionCase(FrenzySelection auto, int armGoTo) {
        this.auto = auto;
        this.armGoTo = armGoTo;
        this.side = auto.getSide();
        this.forcedCase = auto.getForcedCase();
        this.teamCompatible = auto.getTeamCompatible();

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
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1825))
                    .build();
            goToShippingHub = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse.end())
                    .lineToLinearHeading(new Pose2d(18,sign * 35, Math.toRadians(sign * (10))))
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
                    .lineToLinearHeading(new Pose2d( 18, sign * 23, Math.toRadians(sign * (30))))
                    .build();
            alignWithWall = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(-14,sign * 8, Math.toRadians(sign * (-130))))
                    .addTemporalMarker(.4226, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1100))
                    .addTemporalMarker(2, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .build();
            enterWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall.end())
                    .forward(36)
                    .build();
            exitWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(enterWarehouse.end())
                    .back(39)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1825))
                    .build();
            goToShippingHub = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse.end())
                    .lineToLinearHeading(new Pose2d(18,sign * 35, Math.toRadians(sign * (20))))
                    .build();
            alignWithWall2 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub.end())
                    .lineToLinearHeading(new Pose2d(-15,sign * 18, Math.toRadians(sign * (-130))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            enterWarehouse2 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall2.end())
                    .forward(17)
                    .turn(Math.toRadians(sign * (25)))
                    .forward(25)
                    .build();
            exitWarehouse2 = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(enterWarehouse2.end())
                    .back(44)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1825))
                    .build();
            goToShippingHub2 = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse2.end())
                    .lineToLinearHeading(new Pose2d(-6,sign * 68, Math.toRadians(sign * (45)))) // era 34
                    .build();
            alignWithWall3 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub2.end())
                    .lineToLinearHeading(new Pose2d(-25,sign * 18,Math.toRadians(sign * (-100))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            parkingSoft = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall3.end())
                    .strafeRight(7)
                    .forward(23)
                    .strafeLeft(sign * 32)
                    .forward(16)
                    .turn(Math.toRadians(sign * -140))
                    .strafeLeft(sign * 10)
                    .build();
        } else if(teamCompatible == TeamCompatible.NONE) {
            deliverPreloadBox = auto.getMecanumDrive()
                    .trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d( 21, sign * (-31)))
                    .build();
            goToCarousel = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(17,sign * 30,Math.toRadians(sign * (-130))))
                    .build();
            alignWithCarousel = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(goToCarousel.end())
                    .back(8,
                            SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f * sign))
                    .strafeRight(sign * 2.78)
                    .build();
            park = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithCarousel.end())
                    .lineToLinearHeading(new Pose2d(57, sign * 38, Math.toRadians(sign * (-90))))
                    .build();
        }

    }

    public void runAuto() {
        if(teamCompatible == TeamCompatible.SOFT_DELTA) {
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            //while(auto.getRobot().getArmMotor().isBusy());
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(alignWithWall);
            //auto.sleep(50);
            auto.getMecanumDrive().followTrajectory(enterWarehouse);
            //auto.sleep(100);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), false);
            auto.getMecanumDrive().followTrajectory(exitWarehouse);
            auto.getMecanumDrive().followTrajectory(goToShippingHub);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            //auto.sleep(50);
            auto.getMecanumDrive().followTrajectory(alignWithWall2);
            auto.getMecanumDrive().followTrajectorySequence(parkingSoft);
        } else if (teamCompatible == TeamCompatible.BAG_UN_CARRY) {
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            //while(auto.getRobot().getArmMotor().isBusy());
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(alignWithWall);
            //auto.sleep(50);
            auto.getMecanumDrive().followTrajectory(enterWarehouse);
            //auto.sleep(100);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), false);
            auto.getMecanumDrive().followTrajectory(exitWarehouse);
            auto.getMecanumDrive().followTrajectory(goToShippingHub);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(alignWithWall2);
            auto.getMecanumDrive().followTrajectorySequence(enterWarehouse2);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), false);
            auto.getMecanumDrive().followTrajectorySequence(exitWarehouse2);
            auto.getMecanumDrive().followTrajectory(goToShippingHub2);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(alignWithWall3);
            auto.getMecanumDrive().followTrajectorySequence(parkingSoft);
        } else if(teamCompatible == TeamCompatible.NONE) {
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            //while(auto.getRobot().getArmMotor().isBusy());
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(goToCarousel);
            auto.getMecanumDrive().followTrajectorySequence(alignWithCarousel);
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), auto.getRobot().getZeroArm());
            auto.sleep(6000);
            auto.getMecanumDrive().followTrajectory(park);
            AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), 0);
        }

    }

}