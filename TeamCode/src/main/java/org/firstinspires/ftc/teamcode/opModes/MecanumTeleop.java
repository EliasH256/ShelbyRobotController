package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.MecanumBot;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BORD_SPD;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BUTT_SPD;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_MAX_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_MIN_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_NUM_LEVS;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_SPD;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_SPD_DWN;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.POSE_EQUAL;


@Config
@TeleOp(name = "Mecanum")
//@Disabled
public class MecanumTeleop extends InitLinearOpMode
{
    private static final String TAG = "SJH_MTD";

    private double PIXEL_WAIT_DETECTED = 300.00;
    private boolean PixelIntakeLogging = false;

    private ElapsedTime pixelSamplingStop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private void initPreStart() throws InterruptedException
    {
        ShelbyBot.OpModeType prevOpModeType = ShelbyBot.curOpModeType;
        ShelbyBot.curOpModeType = ShelbyBot.OpModeType.TELE;
        dashboard.clearDisplay();

        /* Initialize the hardware variables. */
        RobotLog.dd(TAG, "Initialize robot");


        RobotLog.dd(TAG, "Prev opmode type=%s.", prevOpModeType);
        /* Always initialize the sensors like the IMU and color sensors,
         *  even if you are Auto transitioning from another OpMode
        */
        robot.init(this, chas, true);
        robot.setBcm(LynxModule.BulkCachingMode.MANUAL);

        Pose2d startPose = new Pose2d();
        if(prevOpModeType != ShelbyBot.OpModeType.AUTO)
        {
            if (robot.extenderMotor != null){
                robot.initExMot();
            }
            if(robot.elbowMotor != null)
            {
                robot.initElbMot();
            }
        }
        else
        {
            Point2d autonEndPos = robot.getAutonEndPos();
            double autonEndHdg = robot.getAutonEndHdg();
            startPose = new Pose2d(autonEndPos.getX(), autonEndPos.getY(), autonEndHdg);
            RobotLog.dd(TAG, "Start Aend fHdg %.2f", Math.toDegrees(autonEndHdg));
            RobotLog.dd(TAG, "Start Pos %s", autonEndPos.toString());
        }

        /* Opportunity for a telop assist using Trajectory sequences */

        //route = new PPlayRoute(robot, START_RIGHT, robot.getAlliance());
/*      I thought that this was a timing issue, but putting in a 5 sec wait did not help
        
        ElapsedTime imuWait = new ElapsedTime();
        double imuInitWaitTime = 5.0;

        imuWait.reset();
        RobotLog.dd(TAG, "Waiting for IMU BNO055 Device to iniatilize");
                ;
        while (imuWait.seconds() < imuInitWaitTime)
        {
            idle();
            robot.waitForTick(20);
        }
*/

        mechDrv = (MecanumDriveLRR)(robot.drive);
        mechDrv.setPoseEstimate(startPose);
        mechDrv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for(String k : robot.motors.keySet())
        {
            if (robot.motors.get(k) == null) continue;
            RobotLog.dd(TAG, "Motor mode in teleop = " + k + ":" +
                robot.motors.get(k).getMode());
        }

        lBumperPressed = false;
    }
    public boolean pixelTimerIsRunning = false;
    private double PIXEL_THRESHOLD_CONSTANT = 1.0;


public enum pixelStates{
    NOPIX,
    ONEPIX,
    ONEHPIX,
    ONEDPIX,
    TWOPIX,
    DPIX

}

private pixelStates whichPixel = pixelStates.NOPIX;
public double distanceToPixelBackSensor;
public double distanceToPixelFrontSensor;
public  int pixelSamplesSensor1 = 0;
public  int pixelSamplesSensor2 = 0;

    void processSensors1()
    {

//        dashboard.displayText(1, "PIXELS: " + robot.pixelPieces);
        distanceToPixelBackSensor = ((DistanceSensor)robot.colorSensor).getDistance(DistanceUnit.CM);

        if (distanceToPixelBackSensor <= PIXEL_THRESHOLD_CONSTANT)
        {
            pixelSamplesSensor1++;
            if (PixelIntakeLogging) {
                RobotLog.dd(TAG, "Sensor 1 Pixel samples " + pixelSamplesSensor1);
            }
        }

        if (distanceToPixelFrontSensor <= PIXEL_THRESHOLD_CONSTANT)
        {
            pixelSamplesSensor2++;
            if (PixelIntakeLogging) {
                RobotLog.dd(TAG, "Sensor 2 Pixel samples " + pixelSamplesSensor2);
            }
        }


        if (pixelTimerIsRunning && pixelSamplingStop.milliseconds() < PIXEL_WAIT_DETECTED)
        {
            if (PixelIntakeLogging) {
                RobotLog.dd(TAG, "Waiting after first Sample");
            }
            return;
        }


        if (whichPixel == pixelStates.NOPIX)
        {
            if (PixelIntakeLogging)
            {
                RobotLog.dd(TAG, "No Pixels");
            }
            if (distanceToPixelBackSensor <= PIXEL_THRESHOLD_CONSTANT)
            {
                whichPixel = pixelStates.ONEPIX;
                pixelSamplingStop.reset();
                pixelTimerIsRunning = true;
                robot.pixelPieces++;
                if (PixelIntakeLogging) {
                    RobotLog.dd(TAG, "transition to pixel state 1");
                }
            }
        }
        else if (whichPixel == pixelStates.ONEPIX)
        {
            if (PixelIntakeLogging) {
                RobotLog.dd(TAG, "1 Pixels");
            }
            pixelTimerIsRunning = false;
            if (
                    (distanceToPixelBackSensor <= PIXEL_THRESHOLD_CONSTANT)
               )
            {

                whichPixel = pixelStates.TWOPIX;
                if (PixelIntakeLogging)
                {
                   RobotLog.dd(TAG, "transition to pixel state 2");
                }
            }
        }
        else if (whichPixel == pixelStates.TWOPIX)
        {
            if (PixelIntakeLogging)
            {
                RobotLog.dd(TAG, "2 Pixels");
            }
            if (robot.pixelPieces < 2)
            {
                robot.pixelPieces++;
                if (PixelIntakeLogging)
                {
                    RobotLog.dd(TAG, "In pixel state 2");
                }
                robot.sweeperServo1.moveAtRate(0);
                robot.sweeperServo2.moveAtRate(0);
                gamepad2.rumble(350);
            }

        }

    }

    void processSensors() throws InterruptedException
    {

        if (robot.colorFindDistance() < PIXEL_THRESHOLD_CONSTANT)
        {
            robot.pixelPieces++;
            if (robot.pixelPieces == 2)
            {
                robot.sweeperServo1.moveAtRate(0);
                robot.sweeperServo2.moveAtRate(0);
            }
            else if (robot.pixelPieces > 2)
            {
                robot.pixelPieces = 2;
            }
            if (robot.pixelPieces <= 1)
            {
                TimeUnit.MILLISECONDS.sleep(375);
            }
        }
    }


    private void update()throws InterruptedException
    {
        robot.update();

        l = 0;

        cnts=robot.getCnts();
        vels=robot.getVels();

        poseEstimate = robot.drive.getPoseEstimate();
        processSensors();

        if(robot.elev != null)
        {
            lStr = robot.elev.toString();
        }


    }

    private static final boolean VERBOSE = false;
    private void printTelem()
    {
        String cntStr = String.format(Locale.US,"CNTS: %d %d %d %d",
                cnts[0], cnts[1], cnts[2], cnts[3]);
        String velStr = String.format(Locale.US,"VELS: %d %d %d %d",
                (int)vels[0], (int)vels[1], (int)vels[2], (int)vels[3]);
        String posStr = String.format(Locale.US, "X:%.2f Y:%.2f H:%.2f",
            poseEstimate.getX(), poseEstimate.getY(), Math.toDegrees(poseEstimate.getHeading()));

        dashboard.displayText(l++, posStr);
        dashboard.displayText(l++, cntStr);
        dashboard.displayText(l++, velStr);
        dashboard.displayText(l++, String.format(Locale.US,"L_IN %4.2f L %4.2f", raw_lr, lr));
        dashboard.displayText(l++, String.format(Locale.US,"R_IN %4.2f R %4.2f", raw_fb, fb));
        dashboard.displayText(l++, String.format(Locale.US,"T_IN %4.2f T %4.2f", raw_turn, turn));

        dashboard.displayText(l++, String.format(Locale.US,"Mrk Pos %.2f MvRate %.2f ",
                lastMrkPos, moveAtRate));
        if(null != robot) {
            if (null != robot.elev) {
                dashboard.displayText(l++, String.format(Locale.US, "elevator encoding %d", robot.elev.getCurEnc()));
            }
            if (null!= robot.rearDistSensor){

                dashboard.displayText(l++, String.format(Locale.US, "rear distance %f", robot.rearDistSensor.getDistance(DistanceUnit.CM)));
            }
        }
        dashboard.displayText(l++, String.format(Locale.US, "lyftpowr %4.2f", liftSpd ));


        if(VERBOSE) RobotLog.dd(TAG, "TEL SHT:%.1f ARM:%.1f INT:%.1f DRV:%.1f",
            spinTime, liftTime, intTime, drvTime);
        if(VERBOSE) RobotLog.dd(TAG, "TEL U:%.1f C:%.1f D:%.1f P:%.1f L:%.1f F:%.1f W:%.1f",
            u, c, d, p, L, f, w);
    }

    private void doLogging()
    {
        TelemetryPacket packet = cmu.getTelemetryPacket();
        if(packet == null)
        {
            packet = new TelemetryPacket();
            cmu.setTelemetryPacket(packet);
        }

        //Can put stuff before draw call like:
        // packet.put("pos", pos);
        mechDrv.draw();
    }

    double liftSpd = 0.0;
    int stackLvlCnt = 0;
    boolean joystickUsed = false;
    int targetEncoder = 0;

    private void controlArm()
    {


        if(robot.elbowMotor == null) return;
        boolean start = gpad2.pressed(ManagedGamepad.Button.START);
        double lftPwr = gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y)/2;
        /* Move the Elevator to desired HuB level */
        boolean armLevelUp   = gpad2.just_pressed(ManagedGamepad.Button.D_UP);
        boolean armLevelDown   = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
        if (lftPwr >= -.1 && lftPwr <= .1)
        {
            if(robot.elbowMotor.getMode() == RUN_TO_POSITION)
            {


            }
            else
            {
                if(joystickUsed = true){
                    targetEncoder = robot.elbowMotor.getCurEnc() + 50;
                    joystickUsed = false;
                }
                robot.elbowMotor.moveToCnt(targetEncoder, robot.getElSpd(targetEncoder));
            }

        }
        else
        {
            robot.elbowMotor.setMode(RUN_USING_ENCODER);
            liftSpd = lftPwr;
            joystickUsed = true;

            //robot.elev.moveToCnt(robot.elev.getCurEnc(), RobotConstants.EL_SPD);
        }

        int offset = 1;

        if (armLevelUp)
        {
            if (stackLvlCnt < RobotConstants.EL_NUM_LEVS -1)
            {
                stackLvlCnt++;
                ballRestrictionsOff = true;
                robot.elbowMotor.moveToLevel(stackLvlCnt, RobotConstants.EL_SPD);

            }
            if (stackLvlCnt == 4){
                robot.extendr();
            }else {
                robot.destendr();
            }

        }
        else if (armLevelDown)
        {
            if (stackLvlCnt > 0)
            {
                stackLvlCnt--;
                ballRestrictionsOff = true;
                robot.elbowMotor.moveToLevel(stackLvlCnt, EL_SPD_DWN);

            }
            if (stackLvlCnt == 4){
                robot.extendr();
            }else {
                robot.destendr();
            }

//            dashboard.displayText(13, String.format(Locale.US, "D down Pressed %d",stackLvlCnt));
        }
        if (robot.elbowMotor.getMode() != RUN_TO_POSITION)
        {
            if (
                    (liftSpd <= 0 && robot.elbowMotor.getCurEnc() > EL_MIN_ENCODER ) ||
                            (liftSpd >= 0 && robot.elbowMotor.getCurEnc() < EL_MAX_ENCODER)
            )
            {
                double locSpeedLimit = 1;
//                if(liftSpd <= -.1) {
//                    locSpeedLimit = .5;
//                    if (robot.elbowMotor.getCurEnc() <= 300) {
//                        locSpeedLimit = .3;
//                    } else if (robot.elbowMotor.getCurEnc() <= 200) {
//                        locSpeedLimit = .15;
//                    } else if (robot.elbowMotor.getCurEnc() <= 100) {
//                        locSpeedLimit = .075;
//                    }
//                }

                robot.elbowMotor.moveAtControlRate(RobotConstants.EL_SPD * liftSpd * locSpeedLimit);
            }
            else
            {
                robot.elbowMotor.moveAtControlRate(0);
            }

        }

        double elb = robot.elbowMotor.getCurEnc();
        if (elb>=.65){

        }

        robot.setWristServo(gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y));
            if(gpad2.pressed(ManagedGamepad.Button.D_RIGHT))
            {
                robot.setWristServo(.2);
//                dashboard.displayText(12, String.format(Locale.US, "manually adjusting wrist"));
            }
            if(gpad2.pressed(ManagedGamepad.Button.D_LEFT))
            {
                robot.setWristServo(-.2);
//                dashboard.displayText(12, String.format(Locale.US, "manually adjusting wrist"));
            }
            if(elb >-100 && !wristRestrictionsOff)
            {
                /* Stowed position */
             robot.wristServo.moveTo(.185);
//                robot.wristServo.makeDeadServo();
            }
            else if(elb > -750  && robot.elbowMotor.getCurSpd() >= .1 && wristRestrictionsOff)
            {
//                robot.wristServo.makeDeadServo();
            }
            else if(elb >=-950 && elb <=-850 && !wristRestrictionsOff)
            {
                /* before it hits ground on the way to being stowed */
                robot.wristServo.moveTo(1-.43);
            }
            else if(robot.elbowMotor.getCurEnc() < -950 && robot.elbowMotor.getCurEnc() > -1500 && !wristRestrictionsOff)
            {
                /* Back drop Level 1 */
                robot.wristServo.moveTo(0.630);
            }
            else if(robot.elbowMotor.getCurEnc() < -1500 && robot.elbowMotor.getCurEnc() > -1600  && !wristRestrictionsOff)
            {
                /* Back drop Level 1 */
                robot.wristServo.moveTo(0.650);
            }
            else if(robot.elbowMotor.getCurEnc() < -1600 && robot.elbowMotor.getCurEnc() > -1700  && !wristRestrictionsOff)
            {
                /* Back drop Level 2 */
                robot.wristServo.moveTo(0.725);
            }
            else if(robot.elbowMotor.getCurEnc() < -1700 && robot.elbowMotor.getCurEnc() > -1825  && !wristRestrictionsOff)
            {
                /* Back drop Level 3 with extender all the way out */
                robot.wristServo.moveTo(0.770);
            }
            else if(robot.elbowMotor.getCurEnc() < -1825 && robot.elbowMotor.getCurEnc() > -1950  && !wristRestrictionsOff)
            {
                /* Back drop Level 3 with extender all the way out */
                robot.wristServo.moveTo(0.770);
            }
            else if(robot.elbowMotor.getCurEnc() < -1950 && robot.elbowMotor.getCurEnc() > -2000  && !wristRestrictionsOff)
            {
                /* Back drop Level 3 with extender all the way out */
                robot.wristServo.moveTo(0.935);
            }
            else if(robot.elbowMotor.getCurEnc() < -2000  && !wristRestrictionsOff)
            {
                /* Back drop Level 3 with extender all the way out */
                robot.wristServo.moveTo(0.935);
            }
            //the two else ifs below this comment should always be last
            else if(elb <= -850){
//            robot.wristServo.moveTo(0.5);
            }
            else {
                double a = Math.pow(elb,2)* 0;
                double b = .0003 * elb;
                double c = .73;
                double d = robot.elbowMotor.getCurSpd() * .000;
                robot.wristServo.moveTo( 1-(a + b + c + d));
//                dashboard.displayText(12, String.format(Locale.US, "a(%f) + b(%f) + c(%f) + d(%f)= %f",a,b,c,d,a+b+c+d));

            }

//        dashboard.displayText(8, String.format(Locale.US, "extender motor: %d", robot.extenderMotor.getCurEnc()));
//        dashboard.displayText(15, String.format(Locale.US, "arm level: %d, encoder: %d, power: %f ",robot.armLevel,robot.elbowMotor.getCurEnc(),robot.elbowMotor.getPwr()));
    }





    private Pose2d tempPose = new Pose2d();
    private int goLeft = 0;
    private int goRight = 0;
    private int goBrd = 0;
    private double brdDis;









//    private double getBrdDis(){
//        if (robot.elbowMotor.getCurEnc() < -1600 && robot.elbowMotor.getCurEnc() > -1675){
//            return 17;
//        }else if (robot.elbowMotor.getCurEnc() < -1700 && robot.elbowMotor.getCurEnc() > -1850){
//            return 12;
//        }else if (robot.elbowMotor.getCurEnc() < -1850){
//            return 9;
//        }else {
//            return 20;
//        }
//    }

    private double getBrdDis(){
        if (robot.elbowMotor.getCurEnc() < -1450 && robot.elbowMotor.getCurEnc() > -1550){
            return 16; /* cm */
        }else if (robot.elbowMotor.getCurEnc() < -1550 && robot.elbowMotor.getCurEnc() > -1725){
            return 9; /* cm */
        }else if (robot.elbowMotor.getCurEnc() < -1725){
            return 14; /* cm */
        }else {
            return 20; /* cm */
        }
    }


    private void controlDrive()
    {
        if (robot.motors.size() == 0) return;

        raw_lr =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
        raw_fb = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        raw_turn =  gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X);

        //boolean strt =  gpad1.pressed(ManagedGamepad.Button.START);
        boolean incr = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
        boolean decr = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
        boolean hspd = gpad1.pressed(ManagedGamepad.Button.R_TRIGGER);
        boolean slow = gpad1.pressed(ManagedGamepad.Button.L_TRIGGER);
        boolean dtrn = gpad1.pressed(ManagedGamepad.Button.X_PS4_SQUARE);
        boolean tglF = gpad1.just_pressed(ManagedGamepad.Button.Y_PS4_TRIANGLE);
        boolean drvbrd = gpad1.just_pressed(ManagedGamepad.Button.A_PS4_X) && !gpad1.pressed(ManagedGamepad.Button.START);
        boolean rightOne = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
        boolean leftOne = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);

        if (tglF) useField = !useField;

        lr = ishaper.shape(raw_lr, 0.02);
        fb = ishaper.shape(raw_fb, 0.02);
        turn = ishaper.shape(raw_turn, 0.02);

        if      (incr) dSpd += dStp;
        else if (decr) dSpd -= dStp;
        dSpd = Range.clip(dSpd, 0.0, 1.0);





        Vector2d driveInput;
        if(useField)
        {
            // Rotate input vector by the inverse of current bot heading
            driveInput = new Vector2d(lr, fb).rotated(-poseEstimate.getHeading());
        }
        else
        {
            driveInput = new Vector2d(fb, -lr);
        }

        double maxCPS = RobotConstants.DT_SAF_CPS;
        if(hspd) maxCPS = RobotConstants.DT_MAX_CPS;
        if (slow) maxCPS = RobotConstants.DT_SAF_CPS/3;
        double spdScl = maxCPS/RobotConstants.DT_MAX_CPS;

        driveInput = driveInput.times(spdScl);
        turn = turn * spdScl;
        Pose2d velPose;
        Timer timer = new Timer();
        if(Math.abs(lr)+Math.abs(fb) > .25){
           clearDriverOveride();
        }

        if (rightOne){
            goRight = 1;
            goLeft = 0;
            velPose = new Pose2d(0,BUTT_SPD,Math.toRadians(0));
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    clearDriverOveride();
                }
            };
            timer.schedule(task, 225);
        }else if (leftOne){
            goLeft = 1;
            goRight = 0;
            velPose = new Pose2d(0,-BUTT_SPD,Math.toRadians(0));
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    clearDriverOveride();
                }
            };
            timer.schedule(task, 225);
        }else if(drvbrd && robot.rearDistSensor != null){
            goBrd = goBrd == 1?0:1;
            velPose = new Pose2d(-BORD_SPD,0,Math.toRadians(0));
            brdDis = getBrdDis();
            // timer is a backup, this should stop based on sensors - malachi
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    clearDriverOveride();
                }
            };
            timer.schedule(task, 2000);
        }
        else if(goLeft == 1 || goRight == 1 || goBrd == 1){
            velPose = new Pose2d(-BORD_SPD*goBrd, BUTT_SPD*goRight -BUTT_SPD*goLeft, Math.toRadians(0));
        }else{
            velPose = new Pose2d(driveInput, -turn);
        }

        mechDrv.setWeightedDrivePower(velPose);
    }
    private void clearDriverOveride(){
        goRight = 0;
        goLeft = 0;
    }


    private Pose2d setEndPose(Pose2d p1, double y){
        return new Pose2d(p1.getX(), p1.getY() + y, p1.getHeading());
    }
    private boolean comparePoses(Pose2d p1, Pose2d p2)
    {
        boolean similar = true;
        if(Math.abs(p1.getX()-p2.getX()) > POSE_EQUAL){
            similar = false;
        }
        if(Math.abs(p1.getY()-p2.getY()) > POSE_EQUAL){
            similar = false;
        }
        if(Math.abs(Math.toDegrees(p1.getHeading())-Math.toDegrees(p2.getHeading()))/5 > POSE_EQUAL){
            similar = false;
        }
        return similar;
    }

    double spinTime;
    double liftTime;
    double intTime;
    double drvTime;
    double u, c, d, p, L, f, w;
    private double END_GAME_TIMEOUT = 90;
    private boolean inEndGamePeriod = false;
    private boolean enabledDroneLaunching = false;
    private final ElapsedTime endGameNotificationRumble = new ElapsedTime();
    private final ElapsedTime oTimer = new ElapsedTime();
    private final ElapsedTime opTimer = new ElapsedTime();

    private boolean lBumperPressed;

    private void processControllerInputs()
    {
        gpad2.update();

        /* Rumble in the last 30 sec of match with custom effect - End Game*/
        if (endGameNotificationRumble.seconds() > END_GAME_TIMEOUT)
        {
            gpad2.customRumble(1);
            gpad1.customRumble(1);
            /* Sequence matters Processing Driver inputs gets called after
            * This is where the timer maintenance will be taken care of
            * */
            endGameNotificationRumble.reset();
            setENDGAMETIMOUTSECS(25.0);
            inEndGamePeriod = true;
            enabledDroneLaunching = true;

        }

        /* Rumble in the last 5 sec of match */
        if (
                (endGameNotificationRumble.seconds() > END_GAME_TIMEOUT) &&
                (inEndGamePeriod == true)
           )
        {
            gpad2.customRumble(2);
            gpad1.customRumble(2);
            endGameNotificationRumble.reset();
            inEndGamePeriod = false;
        }

        robot.setExtenderPower(gpad2.value(ManagedGamepad.AnalogInput.R_TRIGGER_VAL)-gpad2.value(ManagedGamepad.AnalogInput.L_TRIGGER_VAL));

        if (stackLvlCnt == 3 && ballRestrictionsOff)
        {
            robot.extenderMotor.setMode(RUN_USING_ENCODER);
            robot.extenderMotor.moveToCnt(3750, 1);
            ballRestrictionsOff = false;

        }
        else if (stackLvlCnt == 4 && ballRestrictionsOff)
        {
            robot.extenderMotor.setMode(RUN_USING_ENCODER);
            robot.extenderMotor.moveToCnt(7500, 1);
            ballRestrictionsOff = false;

        }
        else if (stackLvlCnt != 4 && ballRestrictionsOff)
        {
            robot.extenderMotor.setMode(RUN_USING_ENCODER);
            robot.extenderMotor.moveToCnt(5, 1);
            ballRestrictionsOff = false;

        }



        if(gpad2.just_pressed(ManagedGamepad.Button.L_BUMP))
        {
            robot.toggleBucketServoForward();
            robot.pixelPieces --;
            if (robot.pixelPieces <= 1)
			{
                whichPixel = pixelStates.ONEPIX;
            }

            if (robot.pixelPieces <= 0)
			{
                robot.pixelPieces = 0;
                whichPixel = pixelStates.NOPIX;
            }

            RobotLog.dd(TAG, "Purged Pixel " + String.valueOf(robot.pixelPieces));
        }

        if(gpad2.just_pressed(ManagedGamepad.Button.R_BUMP))
        {
            robot.toggleBucketServoBackward();
        }

        if(gpad2.just_pressed(ManagedGamepad.Button.X_PS4_SQUARE))
        {
            robot.toggleIntakes();
            if(robot.rearDistSensor.getDistance(DistanceUnit.CM) > 30)
            {
                if (robot.extenderMotor.getCurEnc() < 100)
                {
                    robot.elbowMotor.moveToLevel(0, EL_SPD);
                    stackLvlCnt = 0;
                    ballRestrictionsOff = true;
                }
                else if (robot.extenderMotor.getCurEnc() < 3000)
                {
                    robot.elbowMotor.moveToLevel(0, 0.35);
                    stackLvlCnt = 0;
                    ballRestrictionsOff = true;
                }
                else if (robot.extenderMotor.getCurEnc() > 3000)
                {
                    robot.elbowMotor.moveToLevel(0, 0.33);
                    stackLvlCnt = 0;
                    ballRestrictionsOff = true;
                }
            }
        }
        if(gpad2.just_pressed(ManagedGamepad.Button.A_PS4_X))
        {
            Timer timer = new Timer();
            if(robot.rearDistSensor.getDistance(DistanceUnit.CM) > 30 || robot.elbowMotor.getCurEnc() > -800 ) {
                if (robot.extenderMotor.getCurEnc() < 100) {
                    robot.elbowMotor.moveToLevel(1, EL_SPD);
                    stackLvlCnt = 1;
                    ballRestrictionsOff = true;
                } else if (robot.extenderMotor.getCurEnc() < 3000) {
                    robot.elbowMotor.moveToLevel(1, 0.35);
                    stackLvlCnt = 1;
                    ballRestrictionsOff = true;
                } else if (robot.extenderMotor.getCurEnc() > 3000) {
                    robot.elbowMotor.moveToLevel(1, 0.33);
                    stackLvlCnt = 1;
                    ballRestrictionsOff = true;
                }
            }
            robot.sweeperServo1.moveAtRate(-0.5);
            robot.sweeperServo2.moveAtRate(-0.5);
            robot.intakesOn = true;
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    robot.intakesOff();
                }
            };
            timer.schedule(task, 500);
        }

        if(gpad2.just_pressed(ManagedGamepad.Button.B_PS4_CIRCLE) && !gpad2.pressed(ManagedGamepad.Button.START)) {
            if (!ballfinalclim & !ballfinalclimout)
            {
                robot.elbowMotor.moveToCnt(-2400, 1);
                ballfinalclim = true;
            }
            else if (ballfinalclim & !ballfinalclimout & robot.elbowMotor.getCurEnc() < -2300)
            {
                robot.extenderMotor.setMode(RUN_USING_ENCODER);
                robot.extenderMotor.moveToCnt(7500, 1);
                ballfinalclimout = true;
            }
            else if (ballfinalclim & ballfinalclimout)
            {
                ballfinalclim = false;
                ballfinalclimout = false;
                robot.extenderMotor.setMode(RUN_USING_ENCODER);
                robot.extenderMotor.moveToCnt(5, 1);
            }
            else
            {
                robot.elbowMotor.moveToCnt(-2400, 1);
                ballfinalclim = true;

            }
        }
        if(gpad2.pressed(ManagedGamepad.Button.START) && (gpad2.just_pressed(ManagedGamepad.Button.L_BUMP) || gpad2.just_pressed(ManagedGamepad.Button.R_BUMP)))
        {
            robot.wristServo.init(-100,100);
            wristRestrictionsOff = true;
        }
        /* disable triangele button during Teleop to prevent accidental Drone Deployment */
        if(gpad2.just_pressed(ManagedGamepad.Button.Y_PS4_TRIANGLE) && enabledDroneLaunching == true)
        {
            robot.droneLauncherServo.moveTo(1.0);
        }
//        if(gpad2.just_pressed(ManagedGamepad.Button.Y_PS4_TRIANGLE))
//        {
//            robot.droneLauncherServo.moveTo(1.0);
//        }


        controlArm();



        opTimer.reset();
        liftTime = opTimer.milliseconds();
        opTimer.reset();

        intTime = opTimer.milliseconds();
        opTimer.reset();
    }

    private void processDriverInputs()
    {
        gpad1.update();

        controlDrive();
        drvTime = opTimer.milliseconds();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this);
        RobotLog.dd(TAG,"RUNNING INIT IN MecanumTeleop");

        initPreStart();

        dashboard.displayText(0, robot.getName() + " is ready");

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested())
        {
            update();
            doLogging();
            robot.waitForTick(20);
        }

        RobotLog.dd(TAG, "Mecanum_Driver starting");

        opTimer.reset();
        endGameNotificationRumble.reset();
        pixelSamplingStop.reset();

        while (opModeIsActive())
        {
            update();
            u=opTimer.milliseconds();
            processControllerInputs();
            c=opTimer.milliseconds();
            processSensors();
            processDriverInputs();
            d=opTimer.milliseconds();
            processSensors();
            w=opTimer.milliseconds();
        }
    }
    double moveAtRate = 0.0;
    private double lastMrkPos = RobotConstants.MK_ARM_STOW;

    double dSpd = 0.0;
    double dStp = 0.1;

    static final double spdScl = Math.sqrt(2.0);
    Input_Shaper ishaper = new Input_Shaper();

    private boolean useField = false;
    private final MecanumBot robot = new MecanumBot();
    private MecanumDriveLRR  mechDrv;

    double raw_lr;
    double raw_fb;
    double raw_turn;
    double lr;
    double fb;
    double turn;

    Pose2d poseEstimate;

    int[] cnts = {0,0,0,0};
    double[] vels = {0,0,0,0};

    private String lStr = "";
    private String aStr ="";
    private int l = 0;

    boolean wristRestrictionsOff = false;
    boolean ballfinalclim = false;
    boolean ballfinalclimout = false;
    boolean ballRestrictionsOff = false;

    public void setENDGAMETIMOUTSECS(double endGameTimeOutTime)
    {
        this.END_GAME_TIMEOUT = endGameTimeOutTime;
    }
}
