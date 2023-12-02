package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.field.CenterStageRoute;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.CenterStageDetector;
import org.firstinspires.ftc.teamcode.robot.MecanumBot;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.field.Field.StartPos.*;

//@SuppressWarnings("ConstantConditions")
@SuppressWarnings("StatementWithEmptyBody")
@Config
@Autonomous(name="CenterStageAuton", group="Auton")
//@Disabled
public class CenterStageAuton extends InitLinearOpMode // implements FtcMenu.MenuButtons
{
    private static final String TAG = "SJH_PPA";
    private static float delay;
    private static float xOffset;
    /* Allows pausing between each route state - useful for testing trajectory sequences 1 at a time */
    protected static String robotNameAuton;

    protected static RobotConstants.Chassis chasAuton;
    public static Field.AutonDebug autonDebug;

    public CenterStageAuton()
    {
        RobotLog.dd(TAG, "PPAuto CTOR");
        alliance = Field.Alliance.valueOf(PreferenceMgr.getAllianceColor());
        startPos = Field.StartPos.values()[PreferenceMgr.getStartPosition()];
        parkPos = Field.ParkLocation.values()[PreferenceMgr.getParkPosition()];
        delay    = PreferenceMgr.getDelay();
        xOffset  = PreferenceMgr.getXOffset();
        autonDebug  = Field.AutonDebug.values()[PreferenceMgr.getEnableAutonDebug()];
        extraPixelGrabStackSideStart = Field.stacksSideExtraPixelGrab.values()[PreferenceMgr.getExtraPixelGrabOnStackSideStart()];
        pathToFrontStage = Field.PathWayToFrontStage.values()[PreferenceMgr.getPathToFrontStage()];
        pathToBackStage  = Field.PathWayToBackStage.values()[PreferenceMgr.getPathToBackStage()];
        pixelPickUpLoc   = Field.PixelStackLoc.values()[PreferenceMgr.getPixelPickupLocation()];

        PreferenceMgr.logPrefs();
        robotNameAuton = PreferenceMgr.getBotName();
        RobotLog.dd(TAG, "robotname: " + robotNameAuton);

        RobotConstants.Chassis tmpChas = RobotConstants.Chassis.B7252;
        try
        {
            tmpChas = RobotConstants.Chassis.valueOf(robotNameAuton);
            RobotLog.dd(TAG, "Robotname %s", tmpChas);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "Robotname %s invalid. Defaulting to %s", robotName, tmpChas);
        }
        chasAuton = tmpChas;

        RobotConstants.init(chasAuton, alliance, startPos, xOffset);
    }

    private void startMode()
    {
        dashboard.clearDisplay();
        do_main_loop();
    }

    //@SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() //throws InterruptedException
    {
        RobotLog.dd(TAG, "initCommon");
        useOpenCv = true;
        initCommon(this);

        if(CommonUtil.getInstance().hasCam)
        {
            RobotLog.dd(TAG, "We have a camera");
            camera = CommonUtil.camera;
        }

        setup();

        int initCycle = 0;
        int initSleep = 20;
        timer.reset();

        robot.setInInit(true);

        boolean inCamSetup = false;

        ElapsedTime advTmr = new ElapsedTime();
        double advTime = 1.5;

        det.setLogging(false);

        /* Before Play is pressed but after Init is triggered we remain here to perform any house keeping */
        while(!isStopRequested() && !isStarted())
        {
            /* Setting up bot from preference manager to run Auton */
            robot.update();

            gpad1.update();
            double camPos = RobotConstants.CAM_RED_1;
            if(startPos == START_BACKDROP)
            {
                if(alliance == Field.Alliance.BLUE)
                    camPos = RobotConstants.CAM_BLU_1;
            }
            if(startPos == START_STACKS)
            {
                camPos = RobotConstants.CAM_RED_2;
                if(alliance == Field.Alliance.BLUE)
                    camPos = RobotConstants.CAM_BLU_2;
            }

            if(
                  (gpad1.just_pressed(ManagedGamepad.Button.A_PS4_X)) &&
                  (!gpad1.pressed(ManagedGamepad.Button.START))
              )
            {
                advTmr.reset();
                inCamSetup = true;

                if(camera != null)
                {
                    /* OpenCV starts the Webcam and starts capturing images
                    processFrame is called continously until the pipeline is paused
                    */
                    RobotLog.dd(TAG, "Setting image processing pipeline");
                    /* Camera is an OpenCV Camera that instantiated in commonUtil
                       To use to the camera we need to setPipeline to an OpenCV Pipeline -
                       Detector is a child OpenCV Pipeline
                     */
                    camera.setPipeline(det);
                    camera.resumeViewport();
                    det.setPaused(false);
                }
            }

            if(gpad1.just_pressed(ManagedGamepad.Button.B_PS4_CIRCLE) &&
               !gpad1.pressed(ManagedGamepad.Button.START))
            {
                inCamSetup = false;
                if(camera != null)
                {
                    camera.pauseViewport();
                    camera.setPipeline(null);
                    det.setPaused(true);
                }
            }

            if(inCamSetup)
            {
                if(det instanceof CenterStageDetector)
                {
                    CenterStageDetector ffdet = (CenterStageDetector)det;
                    initScanPos = ffdet.getPos();
                    initNumContours = ffdet.getNumContours();

                    if(advTmr.seconds() > advTime)
                    {
                        advTmr.reset();
                        ffdet.toggleStage();
                        //ffdet.advanceStage();
                        camera.showFpsMeterOnViewport(false);

                    }
                }
            }
            dashboard.displayText(2, "Scan: "
                                     + initScanPos + " " + initNumContours);

            if(initCycle % 10 == 0)
            {
                mechDrv.drawRoute();
                StringBuilder motStr = new StringBuilder("ENCs:");
                for (Map.Entry<String, DcMotorEx> e : robot.motors.entrySet())
                {
                    if (e.getValue() == null) continue;
                    motStr.append(" ");
                    motStr.append(e.getKey());
                    motStr.append(":");
                    motStr.append(e.getValue().getCurrentPosition());
                }
            }

            initCycle++;

            robot.waitForTick(initSleep);
        }

        det.setLogging(true);

        det.reset();

        robot.setInInit(false);

        if(!isStopRequested())
        {
            startMode();
        }
        stopMode();
    }

    private void stopMode()
    {
        es.shutdownNow();

        if(camera != null)
        {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    private void setupBotComponents()
    {
        if(robot.elev != null)
        {
            robot.elev.setMode(STOP_AND_RESET_ENCODER);
            robot.elev.setMode(RUN_USING_ENCODER);
            RobotLog.dd(TAG,"Elev encoders reset \n %s", robot.elev.toString());
        }

    }

    /* @setup - Following is performed here
    *      - Builds Driver Station dashboard with values written in Auton Config
    *      - setups a the Auto Transitioner - this automatically launches the Teleop op mode upon exit of Power Play Auton
    *      - Initializes and builds all Trajectories
    *      - Initializes a Detector for image processing and enables cropping
    */
    private void setup()
    {
        dashboard.displayText(0, "PLEASE WAIT - STARTING - CHECK DEFAULTS");

        dashboard.displayText(3, "Pref BOT: " + robotNameAuton);
        dashboard.displayText(4, "Pref Alliance: " + alliance);
        dashboard.displayText(5, "Pref StartPos: " + startPos);

        dashboard.displayText(6, "ExtraPixel Grab SS:  " + extraPixelGrabStackSideStart);
        dashboard.displayText(7, "Park Position:  " + parkPos);
        dashboard.displayText(8, String.format(Locale.US, "Pref Delay: %.2f", delay));
        dashboard.displayText(9, "Pref AutonDebug: " + autonDebug);

        dashboard.displayText(10, "Path to FrontStage:  " + pathToFrontStage);
        dashboard.displayText(11, "Path to BackStage:  " + pathToBackStage);
        dashboard.displayText(12, "PixelPickup Loc:  " + pixelPickUpLoc);


        logData = true;
        RobotLog.ii(TAG, "SETUP");
        dashboard.displayText(0, "INITIALIZING - Please wait");
        robot = new MecanumBot();

        //Since we only have 5 seconds between Auton and Teleop, automatically load
        //teleop opmode
        final String teleopName = "Mecanum";
        RobotLog.dd(TAG, "Setting up auto tele loader : %s", teleopName);
        dashboard.displayText(1, "AutoTrans setup");

        ShelbyBot.curOpModeType = ShelbyBot.OpModeType.AUTO;

        robot.init(this, chas, true);
        if(robot.elbowMotor != null)
        {
            try {
                robot.initElbMot();
            } catch (InterruptedException e) {
                RobotLog.dd(TAG, "Stopped mid init for arm encoder reset");
            }
        }

        RobotConstants.info();

        robot.setBcm(LynxModule.BulkCachingMode.MANUAL);
        mechDrv = (MecanumDriveLRR)(robot.drive);

        setupBotComponents();

        /* Build our Auton Trajectories */
        routeRight = new CenterStageRoute(robot, Route.TeamElement.RIGHT, startPos, parkPos, alliance, extraPixelGrabStackSideStart, pathToFrontStage, pathToBackStage, pixelPickUpLoc);
        routeLeft = new CenterStageRoute(robot, Route.TeamElement.LEFT, startPos, parkPos, alliance, extraPixelGrabStackSideStart, pathToFrontStage, pathToBackStage, pixelPickUpLoc);
        routeCenter = new CenterStageRoute(robot, Route.TeamElement.CENTER, startPos, parkPos, alliance, extraPixelGrabStackSideStart, pathToFrontStage, pathToBackStage,pixelPickUpLoc);

        robot.drive.setPoseEstimate(routeCenter.start);
        ShelbyBot.DriveDir startDdir = ShelbyBot.DriveDir.PUSHER;
        robot.setDriveDir(startDdir);
        initHdg = routeCenter.start.getHeading();
        robot.setInitHdg(initHdg);
        robot.setAlliance(alliance);

        ePose = robot.drive.getPoseEstimate();
        robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
        robot.setAutonEndHdg(ePose.getHeading());

        dashboard.displayText(0, "GYRO CALIBRATING DO NOT TOUCH OR START");
        if (robot.imu != null)
        {
            gyroReady = robot.calibrateGyro();
        }

        dashboard.displayText(0, "READY TO START");
        dashboard.displayText(1, "Robot Init-ed");

        det = new CenterStageDetector(robotName);
        det.setCrop(true);

        setupLogger();
        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        RobotLog.ii(TAG, "STARTPOS %s", startPos);
        RobotLog.ii(TAG, "ALLIANCE %s", alliance);
        RobotLog.ii(TAG, "DELAY    %4.2f", delay);
        RobotLog.ii(TAG, "BOT      %s", robotName);
        RobotLog.dd(TAG, "Robot CPI " + RobotConstants.DT_CPI);
        RobotLog.dd(TAG, "BOTDIR=%s START_DDIR =%s", RobotConstants.DT_DIR, startDdir);

    }


    private void performTrajSeq(TrajectorySequence seq, String seqName)
    {
        if(autonDebug == Field.AutonDebug.ENABLE)
        {
            /* log to the Driver Station dashboard during Auton Debug */
            int lnNum = 5;

            dashboard.displayText(lnNum, String.format(Locale.US, "Traj Seq %s len=%d dur=%.2f",
                    seqName, seq.size(), seq.duration()));

            for (int i = 0; i < seq.size(); i++)
            {
                SequenceSegment seg = seq.get(i);

                dashboard.displayText(lnNum++, String.format(Locale.US, "Seg %d %s to %s in %.2f",
                        i, seg.getStartPose(), seg.getEndPose(), seg.getDuration()));


                if (seg instanceof TrajectorySegment)
                {
                    dashboard.displayText(lnNum++, String.format(Locale.US, "TrajSeg %d %s to %s in %.2f",
                            i, seg.getStartPose(), seg.getEndPose(), seg.getDuration()));

                    for (TrajectoryMarker m : ((TrajectorySegment) seg).getTrajectory().getMarkers()) {
                        dashboard.displayText(lnNum++, String.format(Locale.US, "  marker: time=%.2f", m.getTime()));
                    }
                }
                else
                {
                    if (seg instanceof TurnSegment)
                    {
                        dashboard.displayText(lnNum++, String.format(Locale.US, "TurnSeg %d %s to %s in %.2f",
                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration()));
                    }
                    else if (seg instanceof WaitSegment)
                    {
                        dashboard.displayText(lnNum++, String.format(Locale.US, "WaitSeg %d %s to %s in %.2f",
                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration()));
                    }
                    for (TrajectoryMarker m : seg.getMarkers())
                    {
                        dashboard.displayText(lnNum++, String.format(Locale.US, "  marker: time=%.2f", m.getTime()));
                    }
                }
            }
        }

        /* Waits here until game pad button A is pressed before proceeding to the next sequence
         *  Useful for debugging, especially when the bot is not where you would expect it to be.
         *  */
        while(
                (autonDebug == Field.AutonDebug.ENABLE) &&
                (opModeIsActive())
             )
        {
            gpad1.update();
            if(gpad1.just_pressed(ManagedGamepad.Button.A_PS4_X))
            {
                break;
            }

            robot.waitForTick(20);
        }

        RobotLog.ii(TAG, "Driving trajectorySeq %s at %.2f",
                seqName, startTimer.seconds());

        /* Here is where the drivetrain/route magic happens */
        if (opModeIsActive())
        {
            mechDrv.followTrajectorySequenceAsync(seq);
        }

        while (opModeIsActive() && !isStopRequested() && mechDrv.isBusy())
        {
            robot.update();

            /* Get the current pose each frame and save it to a static variable
             * to allow teleop to know where it is when it starts
             */
            ePose = robot.drive.getPoseEstimate();
            robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
            robot.setAutonEndHdg(ePose.getHeading());

        }
    }

    private void do_main_loop()
    {
        startTimer.reset();
        dl.resetTime();
        camera.setPipeline(det);

        RobotLog.ii(TAG, "STARTING AT %.2f %.2f", startTimer.seconds(), timer.seconds());
        if (logData)
        {
            Pose2d spt = routeCenter.start;
            dl.addField("START");
            dl.addField(initHdg);
            dl.addField(spt.getX());
            dl.addField(spt.getY());
            dl.newLine();
        }
        /* Send Telemetry Data to the Dashboard */
        logFtcDashboard();

        RobotLog.ii(TAG, "Delaying for %4.2f seconds", delay);
        ElapsedTime delayTimer = new ElapsedTime();
        while (opModeIsActive() && delayTimer.seconds() < delay)
        {
            idle();
        }

        RobotLog.ii(TAG, "Done delay");

        RobotLog.ii(TAG, "Action SCAN_IMAGE");

        doScan();

        if (detectedTeamElementPosition == CenterStageDetector.Position.LEFT)
        {
            route = routeLeft;

        }
        else if (detectedTeamElementPosition == CenterStageDetector.Position.RIGHT)
        {
            route = routeRight;

        }
        else
        {
            route = routeCenter;

        }
        try {
            int lnum = 7;
            dashboard.displayText(lnum++, "Start:    " + startPos);
            dashboard.displayText(lnum++, "Park Position:  " + parkPos);
            dashboard.displayText(lnum++, "Team Element:    " + route.teamElement);
        }catch (Exception e){}

        if (route == null)
        {
            RobotLog.ee(TAG, "Route Was Not Initialized!");
        }

        timer.reset();

        /* There is a state for each "chunk" of our overall auton route.
        *  Each state gets assigned a trajectorySequence.
        *  Here, we loop thru each trajectorySequence - starting it asynchronously
        *  with mechDrv.followTrajectorySequenceAsync(seq) below.
        *  Then we loop until the sequence is complete - updating the robot and
        *  saving the current pose each time through the loop.
        *  Note that there also some delay/loops before certain states/sequences
        *  to allow us to safely and asynchronously wait for some condition to get
        *  satisfied (i.e. elevator up, freight intake detected, etc)
        */

        doAutonFromInitTrajectories2();
        doAutonParking();


        RobotLog.dd(TAG, "Finished auton segments at %s", ePose.toString());

        ePose = robot.drive.getPoseEstimate();
        robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
        robot.setAutonEndHdg(ePose.getHeading());
        RobotLog.dd(TAG, "Exiting auton at %s", ePose.toString());
    }

    private void doAutonFromInitTrajectories2()
    {
        int trajNum = 0;
        if(null != route.trajList)
        {
            for (TrajectorySequence tSeq : route.trajList)
            {
                trajNum++;

                String seqName = String.format(Locale.US, "Seq %d",
                        trajNum);
                dashboard.displayText(4, seqName);

                performTrajSeq(tSeq, seqName);
            }
        }
    }

    private void doAutonParking(){
        TrajectorySequence parkSeq = route.parkMap.get(detectedTeamElementPosition);
        if(null != parkSeq) {
            String seqName = "unknown";
            if (CenterStageDetector.Position.CENTER == detectedTeamElementPosition) {
                seqName = "CENTER_PARK";
            } else if (CenterStageDetector.Position.LEFT == detectedTeamElementPosition) {
                seqName = "LEFT_PARK";
            } else /* (PPlayDetector.Position.RIGHT == detectedParkPosition) */
            {
                seqName = "RIGHT_PARK";
            }


            //Temporary
            RobotLog.dd(TAG, "detectedParkPosition, %s", detectedTeamElementPosition.name());
            for (int i = 0; i < parkSeq.size(); i++) {
                SequenceSegment seg = parkSeq.get(i);
                RobotLog.dd(TAG, "Seg %d %s to %s in %.2f",
                        i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());

                if (seg instanceof TrajectorySegment) {
                    Trajectory t = ((TrajectorySegment) seg).getTrajectory();
                    RobotLog.dd(TAG, "TrajSeg %d %s to %s in %.2f",
                            i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());
                    for (TrajectoryMarker m : ((TrajectorySegment) seg).getTrajectory().getMarkers()) {
                        RobotLog.dd(TAG, "  marker: time=%.2f", m.getTime());
                    }
                } else {
                    if (seg instanceof TurnSegment) {
                        TurnSegment ts = (TurnSegment) seg;
                        RobotLog.dd(TAG, "TurnSeg %d %s to %s in %.2f",
                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());
                    } else if (seg instanceof WaitSegment) {
                        WaitSegment ws = (WaitSegment) seg;
                        RobotLog.dd(TAG, "WaitSeg %d %s to %s in %.2f",
                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());
                    }
                }
            }

            dashboard.displayText(4, seqName);
            performTrajSeq(parkSeq, seqName);
        }

    }

    private void doScan()
    {
        RobotLog.dd(TAG, "doScan");

        detectedTeamElementPosition =  getImageTargetPos();
        RobotLog.dd(TAG, "doScan scanPos = %s", detectedTeamElementPosition);

        setScanPoint();

    }

    private void setScanPoint()
    {
        RobotLog.dd(TAG, "Getting scanPoint for %s %s %s",
                alliance, startPos, detectedTeamElementPosition);
    }

    @SuppressWarnings("unused")
    private double angNormalize(double ang)
    {
        double ret = ang;
        while (ret >   180) ret -= 360;
        while (ret <= -180) ret += 360;
        return ret;
    }

    private CenterStageDetector.Position getImageTargetPos()
    {
        if(!opModeIsActive()) return detectedTeamElementPosition;

        if(camera == null)
        {
            return CenterStageDetector.Position.CENTER;
        }
        CenterStageDetector.Position tmpScanPos = CenterStageDetector.Position.NONE;

        double scanTimeout = 0.5;

        ElapsedTime mtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()                    &&
              tmpScanPos == CenterStageDetector.Position.NONE &&
              mtimer.seconds() < scanTimeout)
        {
            RobotLog.dd(TAG, "getScanPos - loop calling det.logDebug");
            if(det instanceof CenterStageDetector)
                tmpScanPos = ((CenterStageDetector) det).getPos();

            if(tmpScanPos == CenterStageDetector.Position.NONE)
                sleep(10);
        }

        camera.setPipeline(null);
        camera.stopStreaming();
        det.saveImages();

        dashboard.displayText(2, "scanPos: " + tmpScanPos);
        RobotLog.dd(TAG, "scanPos = %s", tmpScanPos);
        if (alliance == Field.Alliance.RED && startPos == START_STACKS){
            if (tmpScanPos == CenterStageDetector.Position.NONE)
            {
                RobotLog.dd(TAG, "No image answer found - defaulting to RIGHT");
                dashboard.displayText(6, "No image answer found " + tmpScanPos);
                tmpScanPos = CenterStageDetector.Position.RIGHT;
            }
        }
        else if (alliance == Field.Alliance.BLUE && startPos == START_STACKS){
            if (tmpScanPos == CenterStageDetector.Position.NONE)
            {
                RobotLog.dd(TAG, "No image answer found - defaulting to RIGHT");
                dashboard.displayText(6, "No image answer found " + tmpScanPos);
                tmpScanPos = CenterStageDetector.Position.LEFT;
            }
        }
        else if (alliance == Field.Alliance.BLUE && startPos == START_BACKDROP){
            if (tmpScanPos == CenterStageDetector.Position.NONE)
            {
                RobotLog.dd(TAG, "No image answer found - defaulting to RIGHT");
                dashboard.displayText(6, "No image answer found " + tmpScanPos);
                tmpScanPos = CenterStageDetector.Position.RIGHT;
            }
        }
        else if (alliance == Field.Alliance.RED && startPos == START_BACKDROP){
            if (tmpScanPos == CenterStageDetector.Position.NONE)
            {
                RobotLog.dd(TAG, "No image answer found - defaulting to RIGHT");
                dashboard.displayText(6, "No image answer found " + tmpScanPos);
                tmpScanPos = CenterStageDetector.Position.LEFT;
            }
        }
        else
        {
            if (tmpScanPos == CenterStageDetector.Position.NONE)
            {
                RobotLog.dd(TAG, "No image answer found - defaulting to RIGHT");
                dashboard.displayText(6, "No image answer found " + tmpScanPos);
                tmpScanPos = CenterStageDetector.Position.CENTER;
            }
        }

        return tmpScanPos;
    }

    private void setupLogger()
    {
        if (logData)
        {
            dl.addField("NOTE");
            dl.addField("FRAME");
            dl.addField("Gyro");
            dl.addField("LENC");
            dl.addField("RENC");
            dl.addField("LPWR");
            dl.addField("RPWR");
            dl.addField("RED");
            dl.addField("GRN");
            dl.addField("BLU");
            dl.addField("ESTX");
            dl.addField("ESTY");
            dl.addField("ESTH");
            dl.newLine();
        }
    }



    private void logFtcDashboard()
    {
        TelemetryPacket packet = cmu.getTelemetryPacket();
        if(packet == null)
        {
            packet = new TelemetryPacket();
            cmu.setTelemetryPacket(packet);
        }
        FtcDashboard dbd = CommonUtil.getInstance().getFtcDashboard();
        packet.put("pos", robot.logIntakeCurSpd);
        packet.put("Spd", robot.logIntakeCurSpd);

        dbd.sendTelemetryPacket(packet);
    }

    private MecanumBot robot;
    private MecanumDriveLRR mechDrv;
    private Pose2d ePose = new Pose2d();

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime startTimer = new ElapsedTime();

    private Detector det;
    private CenterStageDetector.Position detectedTeamElementPosition = CenterStageDetector.Position.NONE;
    private CenterStageDetector.Position initScanPos = CenterStageDetector.Position.NONE;
    private int initNumContours = 0;

    private double initHdg = 0.0;
    private boolean gyroReady;

    @SuppressWarnings("unused")
    private final boolean useImageLoc  = false;
    private OpenCvCamera camera;
    private final ExecutorService es = Executors.newSingleThreadExecutor();
    private CenterStageRoute route;
    private CenterStageRoute routeRight;
    private CenterStageRoute routeLeft;
    private CenterStageRoute routeCenter;
}