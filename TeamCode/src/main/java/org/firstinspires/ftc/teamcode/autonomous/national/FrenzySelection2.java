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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name= "Frenzy Selection Bun")
public class FrenzySelection2 extends LinearOpMode {

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
        while (opModeIsActive() && !isStopRequested()) {
            runtime.reset();
            mecanumDrive.setPoseEstimate(new Pose2d(0, 0)); //sterge daca nu merge
            int armGoTo = calculateArmGoTo();
            sleep(1500);
            new SelectionCase2(this, armGoTo).runAuto();
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
        telemetry.addData("", "------Options------");
        telemetry.addData("Side", side.toString() + (cursorOption == 1 ? " <-" : ""));
        telemetry.addData("Forced Case", forcedCase.toString() + (cursorOption == 2 ? " <-" : ""));
        telemetry.addData("Compatible", teamCompatible.toString() + (cursorOption == 3 ? " <-" : ""));
        telemetry.addData("", "------Info------");
        telemetry.addData("Cycle vertical", "DPAD DOWN");
        telemetry.addData("Cycle horizontal", "DPAD RIGHT");
        telemetry.addData("Confirm", "Y");
        telemetry.addData("", "------Detection------");
        telemetry.addData("Case", calculateArmGoTo());
        telemetry.update();
    }

    private int calculateArmGoTo(){
        int armGoTo = 1125;
        if (FrenzyCamera.valRight != 0)
            armGoTo = 1775;
        else if (FrenzyCamera.valLeft != 0)
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

    public static class FrenzyCamera{

        private ElapsedTime runtime = new ElapsedTime();

        //0 means skystone, 1 means yellow stone
        //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
        public static int valMid = 0;
        public static int valLeft = 0;
        public static int valRight = 0;
        private static int valMobile = 0;

        private static float rectHeight = .8f/8f;
        private static float rectWidth = 1.2f/8f;

        private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

        public static double[] leftPos = {0.6/8.0+offsetX, 2.35/8.0+offsetY}; // era 2
        public static double[] midPos = {4.15/8.0+offsetX, 2.6/8.0+offsetY};//0 = col, 1 = row
        public static double[] rightPos = {7.4/8.0+offsetX, 2.5/8.0+offsetY}; // era 6
        public static double[] mobilePos = {4.3/8.0+offsetX, 1.0/8.0+offsetY};
        //moves all rectangles right or left by amount. units are in ratio to monitor

        private final int rows = 1280; //640 x 480
        private final int cols = 720;

        OpenCvCamera phoneCam;

        //detection pipeline
        static class StageSwitchingPipeline extends OpenCvPipeline
        {
            Mat yCbCrChan2Mat = new Mat();
            Mat thresholdMat = new Mat();
            Mat all = new Mat();
            List<MatOfPoint> contoursList = new ArrayList<>();

            enum Stage
            {//color difference. greyscale
                detection,//includes outlines
                THRESHOLD,//b&w
                RAW_IMAGE,//displays raw view
            }

            private org.firstinspires.ftc.teamcode.autonomous.national.FrenzyCamera.StageSwitchingPipeline.Stage stageToRenderToViewport = org.firstinspires.ftc.teamcode.autonomous.national.FrenzyCamera.StageSwitchingPipeline.Stage.detection;
            private org.firstinspires.ftc.teamcode.autonomous.national.FrenzyCamera.StageSwitchingPipeline.Stage[] stages = org.firstinspires.ftc.teamcode.autonomous.national.FrenzyCamera.StageSwitchingPipeline.Stage.values();

            @Override
            public void onViewportTapped()
            {
                /*
                 * Note that this method is invoked from the UI thread
                 * so whatever we do here, we must do quickly.
                 */

                int currentStageNum = stageToRenderToViewport.ordinal();

                int nextStageNum = currentStageNum + 1;

                if(nextStageNum >= stages.length)
                {
                    nextStageNum = 0;
                }

                stageToRenderToViewport = stages[nextStageNum];
            }

            @Override
            public Mat processFrame(Mat input)
            {
                contoursList.clear();
                /*
                 * This pipeline finds the contours of yellow blobs such as the Gold Mineral
                 * from the Rover Ruckus game.
                 */

                //color diff cb.
                //lower cb = more blue = skystone = white
                //higher cb = less blue = yellow stone = grey
                Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
                Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

                //b&w
                Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 110, 255, Imgproc.THRESH_BINARY_INV);

                //outline/contour
                Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                yCbCrChan2Mat.copyTo(all);//copies mat object
                //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


                //get values from frameculcat
                double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
                valMid = (int)pixMid[0];

                double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
                valLeft = (int)pixLeft[0];

                double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
                valRight = (int)pixRight[0];

                double[] pixMobile = thresholdMat.get((int)(input.rows()* mobilePos[1]), (int)(input.cols()* mobilePos[0]));//gets value at circle
                valMobile = (int)pixMobile[0];
                //create three points
                Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
                Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
                Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));
                Point pointMobile = new Point((int)(input.cols()* mobilePos[0]), (int)(input.rows()* mobilePos[1]));


                //draw circles on those points
                Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
                Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
                Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle
                Imgproc.circle(all, pointMobile,5, new Scalar( 255, 0, 0 ),1 );//draws circle

                //draw 3 rectangles
                Imgproc.rectangle(//1-3
                        all,
                        new Point(
                                input.cols()*(leftPos[0]-rectWidth/2),
                                input.rows()*(leftPos[1]-rectHeight/2)),
                        new Point(
                                input.cols()*(leftPos[0]+rectWidth/2),
                                input.rows()*(leftPos[1]+rectHeight/2)),
                        new Scalar(0, 255, 0), 4);
                Imgproc.rectangle(//3-5
                        all,
                        new Point(
                                input.cols()*(midPos[0]-rectWidth/2),
                                input.rows()*(midPos[1]-rectHeight/2)),
                        new Point(
                                input.cols()*(midPos[0]+rectWidth/2),
                                input.rows()*(midPos[1]+rectHeight/2)),
                        new Scalar(0, 255, 0), 4);
                Imgproc.rectangle(//5-7
                        all,
                        new Point(
                                input.cols()*(rightPos[0]-rectWidth/2),
                                input.rows()*(rightPos[1]-rectHeight/2)),
                        new Point(
                                input.cols()*(rightPos[0]+rectWidth/2),
                                input.rows()*(rightPos[1]+rectHeight/2)),
                        new Scalar(0, 255, 0), 4);
                Imgproc.rectangle(//5-7
                        all,
                        new Point(
                                input.cols()*(mobilePos[0]-rectWidth/2),
                                input.rows()*(mobilePos[1]-rectHeight/2)),
                        new Point(
                                input.cols()*(mobilePos[0]+rectWidth/2),
                                input.rows()*(mobilePos[1]+rectHeight/2)),
                        new Scalar(255, 0, 0), 4);

                switch (stageToRenderToViewport)
                {
                    case THRESHOLD:
                    {
                        return thresholdMat;
                    }

                    case detection:
                    {
                        return all;
                    }

                    case RAW_IMAGE:
                    {
                        return input;
                    }

                    default:
                    {
                        return input;
                    }
                }
            }

        }

        public ElapsedTime getRuntimeElapsed() {
            return runtime;
        }

        public int getRows() {
            return rows;
        }

        public int getCols() {
            return cols;
        }

        public static int getValLeft() {
            return valLeft;
        }

        public static int getValMid() {
            return valMid;
        }

        public static int getValRight() {
            return valRight;
        }

        public static int getValMobile() {
            return valMobile;
        }
    }

}
