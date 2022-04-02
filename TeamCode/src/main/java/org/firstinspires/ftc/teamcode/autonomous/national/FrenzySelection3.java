package org.firstinspires.ftc.teamcode.autonomous.national;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.national.option.ForcedCase;
import org.firstinspires.ftc.teamcode.autonomous.national.option.ParkingLocation;
import org.firstinspires.ftc.teamcode.autonomous.national.option.Side;
import org.firstinspires.ftc.teamcode.autonomous.national.option.TeamCompatible;
import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name= "Frenzy Selection f bun")
public class FrenzySelection3 extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    Side side = Side.RED;
    ForcedCase forcedCase = ForcedCase.DETECTION;
    ParkingLocation parkingLocation = ParkingLocation.STORAGE;
    TeamCompatible teamCompatible = TeamCompatible.NONE;

    RobotDefinition robot;
    SampleMecanumDrive mecanumDrive;
    FrenzyCamera frenzyCamera;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        selectOptions();
        waitForStart();
        if (opModeIsActive() && !isStopRequested()) {
            runtime.reset();
            mecanumDrive.setPoseEstimate(new Pose2d(0, 0)); //sterge daca nu merge
            frenzyCamera = new FrenzyCamera();
            int armGoTo = 1125;
            if (frenzyCamera.valRight != 0)
                armGoTo = 1775;
            else if (frenzyCamera.valLeft != 0)
                armGoTo = 650;
            sleep(1500);
            new SelectionCase3(this, armGoTo).runAuto();
            telemetry.addData("Case", armGoTo);
            telemetry.update();
            sleep(30000);
        }
    }

    private void initialize(){
        robot = new RobotDefinition(hardwareMap);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClawOpen(robot, robot.getExcavator(), false);
        frenzyCamera = new FrenzyCamera();
    }

    int cursorOption = 1;
    int navigatingDelay = 600;
    /*
        LISTA OPTIUNI (IDs)
        1 - Parte
        2 - Caz fortat
        3 - Parcare
        4 - Special compatibil
     */

    private void selectOptions(){
        boolean confirmed = false;
        while(!confirmed && !isStopRequested()){
            cycleOptions();
            showcaseOptions();
            if(gamepad1.y)
                confirmed = true;
            sleep(navigatingDelay);
        }
        telemetry.addData("", "Options confirmed. Ready for start.");
        telemetry.addData("", "(19121 nu e soft deloc)");
        telemetry.update();
    }

    private void cycleOptions(){
        if(gamepad1.dpad_down){
            cursorOption++;
            if(cursorOption == 4)
                cursorOption = 1;
        }
        if(gamepad1.dpad_right){
            switch(cursorOption){
                case 1:
                    int newSide = side.ordinal() + 1;
                    if(newSide == Side.values().length)
                        newSide = 0;
                    side = Side.values()[newSide];
                    break;
                case 2:
                    int newCase = forcedCase.ordinal() + 1;
                    if(newCase == ForcedCase.values().length)
                        newCase = 0;
                    forcedCase = ForcedCase.values()[newCase];
                    break;
                case 3:
                    int newCompatible = teamCompatible.ordinal() + 1;
                    if(newCompatible == TeamCompatible.values().length)
                        newCompatible = 0;
                    teamCompatible = TeamCompatible.values()[newCompatible];
                    break;
            }
        }
    }

    private void showcaseOptions(){
        telemetry.addData("", "------Detection------");
        telemetry.addData("Case", calculateArmGoTo());
        telemetry.addData("", "------Options------");
        telemetry.addData("Side", side.toString() + (cursorOption == 1 ? " <-" : ""));
        telemetry.addData("Forced Case", forcedCase.toString() + (cursorOption == 2 ? " <-" : ""));
        telemetry.addData("Compatible", teamCompatible.toString() + (cursorOption == 3 ? " <-" : ""));
        telemetry.addData("", "------Info------");
        telemetry.addData("Cycle vertical", "DPAD DOWN");
        telemetry.addData("Cycle horizontal", "DPAD RIGHT");
        telemetry.addData("Confirm", "Y");
        telemetry.update();
    }

    private int calculateArmGoTo(){
        frenzyCamera = new FrenzyCamera();
        int armGoTo = 1125;
        if (frenzyCamera.valRight != 0)
            armGoTo = 1775;
        else if (frenzyCamera.valLeft != 0)
            armGoTo = 650;
        return armGoTo;
    }

    public SampleMecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public RobotDefinition getRobot() {
        return robot;
    }

    public Side getSide() {
        return side;
    }

    public ForcedCase getForcedCase() {
        return forcedCase;
    }

    public ParkingLocation getParkingLocation() {
        return parkingLocation;
    }

    public TeamCompatible getTeamCompatible() {
        return teamCompatible;
    }

    public ElapsedTime getElapsed() {
        return runtime;
    }

}
