package org.firstinspires.ftc.teamcode.util;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.field.Field;

@SuppressWarnings("unused")
public class PreferenceMgr
{
   private static final String TAG = "SJH_PRF";
   private static SharedPreferences prefs;
   private static final String CLUBNAME = "Shelby";

   private static String botName;
   private static String alliance;
   private static int    startPos;
   private static int autonStrgy;
   private static int    parkPos;
   private static float  delay;
   private static float  xOffset;
   private static int  autonDebug;
   private static int extraPixelGrabOnStackSideStart;
   private static int pathToFrontStage;
   private static int pathToBackStage;
   private static int pixelStackPickLocation;



   static
   {
      RobotLog.dd(TAG, "Init static block");
      RobotLog.dd(TAG, "applicationID=%s", AppUtil.getInstance().getApplicationId());
      prefs = PreferenceManager.getDefaultSharedPreferences(AppUtil.getInstance().getApplication());
      readPrefs();
      writePrefs(); /* This will write defaults if prefs were empty at init */
   }

   public static String getClubName() { return CLUBNAME; }
   public static int getParkPosition() { return parkPos; }
   public static int getExtraPixelGrabOnStackSideStart() { return extraPixelGrabOnStackSideStart; }
   public static String getBotName()  { return botName; }
   public static String getAllianceColor() { return alliance; }
   public static int getStartPosition() { return startPos; }
   public static int getAutonStrategy() { return autonStrgy; }
   public static float getDelay() { return delay; }
   public static float getXOffset() { return xOffset; }
   public static int getEnableAutonDebug() { return autonDebug; }
   public static int  getPathToFrontStage() {  return pathToFrontStage;}
   public static int  getPathToBackStage() { return pathToBackStage; }
   public static int  getPixelPickupLocation() { return pixelStackPickLocation; }

   public void setBotName(String botName) { PreferenceMgr.botName = botName; }
   public void setAllianceColor(String allianceColor) { PreferenceMgr.alliance = allianceColor; }
   public void setStartPosition(int startPosition) { PreferenceMgr.startPos = startPosition; }
   public void setAutonStrategy(int strategy) { PreferenceMgr.autonStrgy = strategy; }
   public void setParkPosition(int parkPos)   { PreferenceMgr.parkPos = parkPos; }
   public void setDelay(float delay) { PreferenceMgr.delay =  delay; }
   public void setXOffset(float offset) { PreferenceMgr.xOffset =  offset; }
   public void setEnableAutonDebug(int debugEnable) { PreferenceMgr.autonDebug =  debugEnable; }
   public void setPathToFrontStage(int routeToFrontStage) { PreferenceMgr.pathToFrontStage = routeToFrontStage;}
   public void setPathToBackStage(int routeToBackStage) { PreferenceMgr.pathToBackStage = routeToBackStage;}
   public void setPixelPickupLocation(int pixelPickupLoc) { PreferenceMgr.pixelStackPickLocation = pixelPickupLoc;}
   public void setExtraPixelGrab(int FirstLoc) { PreferenceMgr.extraPixelGrabOnStackSideStart =  FirstLoc; }


   public PreferenceMgr()
   {
   }

   private static void readPrefs()
   {
      try
      {
          botName  = prefs.getString(CLUBNAME + ".botName", "B7252");
          alliance = prefs.getString(CLUBNAME + ".alliance", "RED");
          startPos = prefs.getInt(   CLUBNAME + ".startPos",1);
          autonStrgy = prefs.getInt(CLUBNAME + ".autonStrgy", 1);
          extraPixelGrabOnStackSideStart = prefs.getInt(CLUBNAME + ".firstLoc", 1);
          delay    = prefs.getFloat( CLUBNAME + ".delay", 0.0f);
          xOffset  = prefs.getFloat( CLUBNAME + ".xOffset", 0.0f);
          autonDebug  = prefs.getInt( CLUBNAME + ".autonDebug", 1);
          pathToFrontStage = prefs.getInt(CLUBNAME + ".pathToFrontStage", 1);
          pathToBackStage = prefs.getInt(CLUBNAME + ".pathToBackStage", 1);
          pixelStackPickLocation = prefs.getInt(CLUBNAME + ".pixelStackPickLocation", 1);
      }
      catch(Exception e)
      {
          RobotLog.dd(TAG, "unable to read prefs or sum like that");
      }

   }

   public static void writePrefs()
   {
      try
      {
         //write the options to sharedpreferences
         SharedPreferences.Editor editor = prefs.edit();
         editor.putString(CLUBNAME + ".botName", botName);
         editor.putString(CLUBNAME + ".alliance", alliance);
         editor.putInt(CLUBNAME + ".startPos", startPos);
         editor.putInt(CLUBNAME + ".autonStrgy", autonStrgy);
         editor.putInt(CLUBNAME + ".parkPos", parkPos);
         editor.putFloat(CLUBNAME + ".delay", delay);
         editor.putFloat(CLUBNAME + ".xOffset", xOffset);
         editor.putInt(CLUBNAME + ".extraPixelGrabOnStackSideStart", extraPixelGrabOnStackSideStart);
         editor.putInt(CLUBNAME + ".autonDebug", autonDebug);
         editor.putInt(CLUBNAME + ".pathToFrontStage", pathToFrontStage);
         editor.putInt(CLUBNAME + ".pathToBackStage", pathToBackStage);
         editor.putInt(CLUBNAME + ".pixelStackPickLocation", pixelStackPickLocation);
         editor.apply();

         prefs = PreferenceManager.getDefaultSharedPreferences(AppUtil.getInstance().getApplication());
      }
      catch (Exception e)
      {
         RobotLog.dd(TAG, "unable to write prefs or sum like that");
      }

   }

   public static void logPrefs()
   {
      try
      {
          RobotLog.dd(TAG, "Default Config Values:");
          RobotLog.dd(TAG, "Club:     %s", CLUBNAME);
          RobotLog.dd(TAG, "Bot:      %s", botName);
          RobotLog.dd(TAG, "Alliance: %s", alliance);
          RobotLog.dd(TAG, "startPos: %d", startPos);
          RobotLog.dd(TAG, "autonStrgy:  %d", autonStrgy);
          RobotLog.dd(TAG, "delay:    %4.1f", delay);
          RobotLog.dd(TAG, "xOffset:  %4.1f", xOffset);
          RobotLog.dd(TAG, "extraPixelGrabOnStackSideStart:  %d", extraPixelGrabOnStackSideStart);
          RobotLog.dd(TAG, "AutonDebug:  %d", autonDebug);
          RobotLog.dd(TAG, "PathWay to Front Stage: %d", pathToFrontStage);
          RobotLog.dd(TAG, "PathWay to Back Stage: %d", pathToBackStage);
          RobotLog.dd(TAG, "PixelPick Location: %d", pixelStackPickLocation);
      }
      catch (Exception e)
      {
          RobotLog.dd(TAG, "unable to log prefs or sum like that");
      }
   }
}