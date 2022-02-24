package org.firstinspires.ftc.teamcode.autonomous.national;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.national.option.ForcedCase;
import org.firstinspires.ftc.teamcode.autonomous.national.option.Side;
import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name= "Frenzy Selection")
public class FrenzySelection extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    Side side = Side.RED;
    ForcedCase forcedCase = ForcedCase.DETECTION;

    RobotDefinition robot;
    SampleMecanumDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        selectOptions();
        runtime.reset();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            new SelectionCase(this, 500).runAuto();
        }
    }

    private void initialize(){
        robot = new RobotDefinition(hardwareMap);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClawOpen(robot.getExcavator(), true);
    }

    int cursorOption = 1;
    int navigatingDelay = 1500;
    /*
        LISTA OPTIUNI (IDs)
        1 - Parte
        2 - Caz fortat
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
            if(cursorOption == 3)
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
                    side = Side.values()[newCase];
                    break;
            }
        }
    }

    private void showcaseOptions(){
        telemetry.addData("", "------Options------");
        telemetry.addData("Side", side.toString() + (cursorOption == 1 ? " <-" : ""));
        telemetry.addData("Forced Case", forcedCase.toString() + (cursorOption == 2 ? " <-" : ""));
        telemetry.addData("", "------Info------");
        telemetry.addData("Cycle vertical", "DPAD DOWN");
        telemetry.addData("Cycle horizontal", "DPAD RIGHT");
        telemetry.addData("Confirm", "Y");
        telemetry.update();
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

}