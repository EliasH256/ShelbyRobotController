package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

import org.firstinspires.ftc.teamcode.util.FtcChoiceMenu;
import org.firstinspires.ftc.teamcode.util.FtcMenu;
import org.firstinspires.ftc.teamcode.util.FtcValueMenu;

@Autonomous(name = "Auton Config", group = "0")
public class AutonConfig extends InitLinearOpMode implements FtcMenu.MenuButtons {

    private static final String TAG = "Auton Menu";

    /* The autonomous menu settings using sharedpreferences */
    private final PreferenceMgr prfMgr = new PreferenceMgr();
    private final static String club;
    private static RobotConstants.Chassis bot;
    private static Field.Alliance allianceColor;
    private static Field.StartPos startPosition;
    private static float delay;
    private static float xOffset;
    private static Field.AutonDebug autonDebugEnable;
    private static Field.ParkLocation parkPos;
    private static Field.stacksSideExtraPixelGrab extrPxlGrb;
    /* Select which path to take from the Canvas back to the Pixel Stack */
    private static Field.PathWayToFrontStage routetoFrontStageFromCanvas;
	
	/* Select which path to take from the Stacks back to the Canvas */
    private static Field.PathWayToBackStage routetoBackStageFromPixelStacks;

    /* Select Pixel Stack to select from */
    private static Field.PixelStackLoc pixelStackLocationPickup;

    static
    {
        club = PreferenceMgr.getClubName();
        getPrefs();
    }
    private int lnum = 1;

    public AutonConfig()
    {
        RobotLog.dd(TAG, "AutonConfig ctor");
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this);
        dashboard.displayText(0, "Starting Menu System");
        setup();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            idle();
        }
    }


    private void setup()
    {
        dashboard.displayText(0, "INITIALIZING - Please wait for Menu");
        doMenus();
        dashboard.displayText(0, "COMPLETE - Settings Written");
    }

    private static void getPrefs()
    {
        try
        {
            bot = RobotConstants.Chassis.valueOf(PreferenceMgr.getBotName());
        }
        catch (Exception e)
        {
            bot = RobotConstants.Chassis.values()[0];
        }
        try
        {
            allianceColor = Field.Alliance.valueOf(PreferenceMgr.getAllianceColor());
        }
        catch(Exception e)
        {
            allianceColor = Field.Alliance.values()[0];
        }
        try
        {
            startPosition = Field.StartPos.values()[PreferenceMgr.getStartPosition()];
        }
        catch(Exception e)
        {
            startPosition = Field.StartPos.values()[0];
        }

        try
        {
            autonDebugEnable = Field.AutonDebug.values()[PreferenceMgr.getEnableAutonDebug()];
        }
        catch(Exception e)
        {
            autonDebugEnable = Field.AutonDebug.values()[0];
        }
        try
        {
            parkPos = Field.ParkLocation.values()[PreferenceMgr.getParkPosition()];
        }
        catch(Exception e)
        {
            parkPos = Field.ParkLocation.values()[0];
        }
        try
        {
            extrPxlGrb = Field.stacksSideExtraPixelGrab.values()[PreferenceMgr.getExtraPixelGrabOnStackSideStart()];
        }
        catch(Exception e)
        {
            extrPxlGrb = Field.stacksSideExtraPixelGrab.values()[0];
        }

        try
        {
            routetoBackStageFromPixelStacks = Field.PathWayToBackStage.values()[PreferenceMgr.getPathToBackStage()];
        }
        catch(Exception e)
        {
            routetoBackStageFromPixelStacks = Field.PathWayToBackStage.values()[0];
        }

        try
        {
            routetoFrontStageFromCanvas = Field.PathWayToFrontStage.values()[PreferenceMgr.getPathToFrontStage()];
        }
        catch(Exception e)
        {
            routetoFrontStageFromCanvas = Field.PathWayToFrontStage.values()[0];
        }

        try
        {
            pixelStackLocationPickup = Field.PixelStackLoc.values()[PreferenceMgr.getPixelPickupLocation()];
        }
        catch(Exception e)
        {
            pixelStackLocationPickup = Field.PixelStackLoc.values()[0];
        }

        delay         = PreferenceMgr.getDelay();
        xOffset       = PreferenceMgr.getXOffset();

        PreferenceMgr.logPrefs();
    }

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()   { return gamepad1.dpad_up;}

    @Override
    public boolean isMenuAltUpButton()
    {
        return gamepad1.left_bumper;
    }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuAltDownButton()
    {
        return gamepad1.right_bumper;
    }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.a; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    private static final boolean showBotMenu = true;
    private void doMenus()
    {
        FtcChoiceMenu<RobotConstants.Chassis> botMenu
            = new FtcChoiceMenu<>("Bot:",      null,         this);
        FtcMenu allianceParent = null;
        if(showBotMenu) allianceParent = botMenu;
        FtcChoiceMenu<Field.Alliance> allianceMenu
            = new FtcChoiceMenu<>("Alliance:", allianceParent,      this);
        FtcMenu topMenu = botMenu;
        if(!showBotMenu) topMenu = allianceMenu;
        FtcChoiceMenu<Field.StartPos> startPosMenu
            = new FtcChoiceMenu<>("START:", allianceMenu, this);
        FtcValueMenu  delayMenu
            = new FtcValueMenu("Delay:",       startPosMenu,     this,
            0.0, 20.0, 1.0, delay, "%5.2f");
        FtcValueMenu  xOffsetMenu
          = new FtcValueMenu("xOffset:",       delayMenu,     this,
            0.0, 12.0, 1.0, xOffset, "%5.2f");
        FtcChoiceMenu<Field.AutonDebug> autoDebugMenu
                = new FtcChoiceMenu<>("AUTON DEBUG:",   xOffsetMenu, this);
        FtcChoiceMenu<Field.ParkLocation> parkPosMenu
                = new FtcChoiceMenu<>("Park Position:",   autoDebugMenu, this);
        FtcChoiceMenu<Field.stacksSideExtraPixelGrab> extraPixelGrabMenu
                = new FtcChoiceMenu<>("Extra Pixel Grab:",   parkPosMenu, this);
        FtcChoiceMenu<Field.PathWayToFrontStage> pathWayToFrontStageMenu
                = new FtcChoiceMenu<>("Bonus 2 Pixels: Path to Front Stage From Canvas:",   extraPixelGrabMenu, this);
        FtcChoiceMenu<Field.PathWayToBackStage> pathWayToBackStageMenu
                = new FtcChoiceMenu<>("Bonus 2 Pixels: Path to Back Stage From Pixel Stacks:",   pathWayToFrontStageMenu, this);
        FtcChoiceMenu<Field.PixelStackLoc> pixelPickLocationMenu
                = new FtcChoiceMenu<>("Bonus 2 Pixels: Pixel Pick Up Locations:",   pathWayToBackStageMenu, this);


        //
        // remember last saved settings and reorder the menu with last run settings as the defaults
        //

        for(RobotConstants.Chassis b : RobotConstants.Chassis.values())
        {
            botMenu.addChoice(b.toString(), b, b==bot, allianceMenu);
        }

        for(Field.Alliance a : Field.Alliance.values())
        {
            allianceMenu.addChoice(a.toString(), a, a==allianceColor, startPosMenu);
        }

        for(Field.StartPos p : Field.StartPos.values())
        {
            startPosMenu.addChoice(p.toString(), p, p==startPosition, extraPixelGrabMenu);
        }

        for(Field.stacksSideExtraPixelGrab f : Field.stacksSideExtraPixelGrab.values())
        {
            extraPixelGrabMenu.addChoice(f.toString(), f, f == extrPxlGrb, parkPosMenu);
        }
        for(Field.ParkLocation e : Field.ParkLocation.values())
        {
            parkPosMenu.addChoice(e.toString(), e, e == parkPos, delayMenu);
        }

        delayMenu.setChildMenu(xOffsetMenu);
        xOffsetMenu.setChildMenu(autoDebugMenu);

        for(Field.AutonDebug d : Field.AutonDebug.values())
        {
            autoDebugMenu.addChoice(d.toString(), d, d== autonDebugEnable, pathWayToFrontStageMenu);
        }

        for(Field.PathWayToFrontStage d : Field.PathWayToFrontStage.values())
        {
            pathWayToFrontStageMenu.addChoice(d.toString(), d, d== routetoFrontStageFromCanvas , pathWayToBackStageMenu);
        }

        for(Field.PathWayToBackStage d : Field.PathWayToBackStage.values())
        {
            pathWayToBackStageMenu.addChoice(d.toString(), d, d== routetoBackStageFromPixelStacks , pixelPickLocationMenu);
        }

        for(Field.PixelStackLoc d : Field.PixelStackLoc.values())
        {
            pixelPickLocationMenu.addChoice(d.toString(), d, d== pixelStackLocationPickup , null);
        }

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(topMenu, this);
        //
        // Set choices variables.
        //

        if(showBotMenu)
        {
            bot = botMenu.getCurrentChoiceObject();
        }
        startPosition = startPosMenu.getCurrentChoiceObject();
        allianceColor = allianceMenu.getCurrentChoiceObject();
        delay = (float)delayMenu.getCurrentValue();
        xOffset = (float)xOffsetMenu.getCurrentValue();
        autonDebugEnable = autoDebugMenu.getCurrentChoiceObject();
        parkPos = parkPosMenu.getCurrentChoiceObject();
        routetoFrontStageFromCanvas = pathWayToFrontStageMenu.getCurrentChoiceObject();
        routetoBackStageFromPixelStacks = pathWayToBackStageMenu.getCurrentChoiceObject();
        pixelStackLocationPickup = pixelPickLocationMenu.getCurrentChoiceObject();
//        extrPxlGrb = Field.stacksSideExtraPixelGrab.NO_EXTRA_PIXEL;
        extrPxlGrb = extraPixelGrabMenu.getCurrentChoiceObject();



        //
        // Set choices variables.
        //

        RobotLog.dd(TAG, "Writing Config Values:");
        printConfigToLog();

        prfMgr.setBotName(bot.toString());
        prfMgr.setStartPosition(startPosition.ordinal());
        prfMgr.setParkPosition(parkPos.ordinal());
        prfMgr.setAllianceColor(allianceColor.toString());
        prfMgr.setDelay(delay);
        prfMgr.setXOffset(xOffset);
        prfMgr.setEnableAutonDebug(autonDebugEnable.ordinal());
        prfMgr.setExtraPixelGrab(extrPxlGrb.ordinal());
		/* Bonus 2 Pixels Configuration */
        prfMgr.setPathToFrontStage(routetoFrontStageFromCanvas.ordinal());
        prfMgr.setPathToBackStage(routetoBackStageFromPixelStacks.ordinal());
        prfMgr.setPixelPickupLocation(pixelStackLocationPickup.ordinal());
        /* write the options to sharedpreferences */
        PreferenceMgr.writePrefs();

        /* read them back to ensure they were written */
        getPrefs();

        RobotLog.dd(TAG, "Returned Config Values:");
        printConfigToLog();

        dashboard.displayText(lnum++, "Bot:      " + bot);
        dashboard.displayText(lnum++, "Alliance: " + allianceColor);
        dashboard.displayText(lnum++, "Start:    " + startPosition);
        dashboard.displayText(lnum++, "Delay:    " + delay);
        dashboard.displayText(lnum++, "Autonomous Debug:  " + autonDebugEnable);
        dashboard.displayText(lnum++, "Park Position:  " + parkPos);
        dashboard.displayText(lnum++, "Extra Pixel Grab:  " + extrPxlGrb);
        dashboard.displayText(lnum++, "Path to Front Stage:  " + routetoFrontStageFromCanvas);
        dashboard.displayText(lnum++, "Path to Back Stage:  " + routetoBackStageFromPixelStacks);
        dashboard.displayText(lnum++, "Pixel Pickup Location:  " + pixelStackLocationPickup);

    }

    public void printConfigToLog()
    {
        RobotLog.dd(TAG, "Club:     %s", club);
        RobotLog.dd(TAG, "Bot:      %s", bot);
        RobotLog.dd(TAG, "Alliance: %s", allianceColor);
        RobotLog.dd(TAG, "startPos: %s", startPosition);
        RobotLog.dd(TAG, "parkPos:  %s", parkPos);
        RobotLog.dd(TAG, "delay:    %4.1f", delay);
        RobotLog.dd(TAG, "xOffset:  %4.1f", xOffset);
        RobotLog.dd(TAG, "Autonomous Debug:  %s", autonDebugEnable);
        RobotLog.dd(TAG, "Extra Pixel Grab:  %s", extrPxlGrb);
        RobotLog.dd(TAG, "Path to Front Stage:  " + routetoFrontStageFromCanvas);
        RobotLog.dd(TAG, "Path to Back Stage:  " + routetoBackStageFromPixelStacks);
        RobotLog.dd(TAG, "Pixel Pickup Location:  " + pixelStackLocationPickup);
    }

}
