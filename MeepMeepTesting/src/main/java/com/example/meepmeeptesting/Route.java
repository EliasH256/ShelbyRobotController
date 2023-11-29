package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.sequencesegment.TurnSegment;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.sequencesegment.WaitSegment;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

import static com.example.meepmeeptesting.Field.StartPos.*;
import static com.example.meepmeeptesting.RobotConstants.TrajEnum.*;

@SuppressWarnings("unused")
public abstract class Route
{
    public final static TrajectoryVelocityConstraint defVelLim;
    public final static TrajectoryAccelerationConstraint defAccelLim;

    public final static TrajectoryVelocityConstraint wobVelLim;
    public final static TrajectoryAccelerationConstraint wobAccelLim;

    static
	{
        defVelLim = RobotConstants.defVelConstraint;
        defAccelLim = RobotConstants.defAccelConstraint;
        wobVelLim = RobotConstants.slwVelConstraint;
        wobAccelLim = RobotConstants.slwAccelConstraint;
        System.out.println("Init static block");
     }

    protected static final boolean strafeDropX = false;



    public Route(PositionOption startPos,
                 Field.Highways parkPos,
                 Field.Alliance alliance,
                 Field.Route    autonRouteStrategy,
                 Field.FirstLocation firstLocation,
                 TeamElement teamElement,
                 Field.Highways[] highways,
                 Field.Highways[] pixelStacks)
    {
           this.startPos = startPos;
           this.parkPos  = parkPos;
           this.alliance = alliance;
           this.routeStrategy = autonRouteStrategy;
           this.teamElement = teamElement;
           this.highways = highways;
		   this.pixelStacks = pixelStacks;
           this.firstLocation = firstLocation;

           botLen = RobotConstants.BOT_LEN;
           botWid = RobotConstants.BOT_WID;

           double rightSideLineUpToBorderAdjustment = 0.5;


         botBackToCtr = botLen / 2.0;
         botSideToCtr = botWid / 2.0;

         hubRad = 9.0;
         double backOffset = 0.0;

         totalDur = 0;

         /* All points of interests will be based off of quadrant I - positive x, positive y */
         /* Reflection over the x-axis will be done for the alliance side */
         /* Reflection over the y-axis will be done on which side you line up */
         /* Blue Left = Quad I, Blue Right = Quad II, Red Right =  */
         /* For headings: Counter clockwise is positive, Clockwise is negative */
         if (alliance == Field.Alliance.BLUE)
		 {
             /* y scalar value  */
             /* Blue side is in quadrant I & II of the 2D coordinate system */
             sy = 1;
             /* Bot should point south or towards the negative Y direction */
             /* point the bot in the clockwise direction */
             sh = Math.toRadians(0);
             sf = 0;
             sx = -1;
             if (startPos == START_BACKDROP)
			 {
                 /* Blue Right quadrant II */
                 sf = 1;
                 sr = 1;
             }
			 else
			 {
                 sr = 0;
             }
         }
		 else
		 {
             /* Red side is in quadrant III & IV of the 2D coordinate system */
             sy = 1;
             /* Bot should point north or in the positive Y direction on red side */
             /* point the bot in the counter clockwise direction */
             sh = 1;
             sf = -1;
             sx =1;
             flip = Math.toRadians(0);
             if (startPos == START_STACKS)
			 {
                 /* Red Right quadrant IV */
                 sf = 0;
                 sr = 1;
             }
		     else
		     {
                 /* Red Left quadrant III */
                 sr = 0;

             }
         }

         if (startPos == START_BACKDROP)
         {
             strtY = 0.5 * CenterStageField.tileWidth;
          }
         else
         {
             strtY = -1.5 * CenterStageField.tileWidth;
         }

         strtX =  3.0f * CenterStageField.tileWidth - botBackToCtr;

         strtH = Math.toRadians(180.0);

         start = new Pose2d(sx * strtX, sy * strtY, flip + sh* strtH);
         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /* Parks */
        parkDoor = new Pose2d(sx * 5, 50, flip + sh * Math.toRadians(270));
        parkWall = new Pose2d(sx * 63, 50, flip + sh * Math.toRadians(270));

        parkDoorAdj = new Pose2d(sx * 5, 45, flip + sh * Math.toRadians(270));
        parkWallAdj = new Pose2d(sx * 63, 45, flip + sh * Math.toRadians(270));

        parkDoorRed = new Pose2d(sx * 8, 52, flip + sh * Math.toRadians(270));
        parkWallRed = new Pose2d(sx * 67, 55, flip + sh * Math.toRadians(270));
        parkDoorAdjRed = new Pose2d(sx * 8, 46, flip + sh * Math.toRadians(270));
        parkWallAdjRed = new Pose2d(sx * 67, 48, flip + sh * Math.toRadians(270));

        parkWallRedRight = new Pose2d(sx * 49, 63, flip + sh * Math.toRadians(270));
        /* Parks */

        /* Qualifier Route Points */
        moveAwayFromWallRedBackdrop = new Pose2d(50, 15, Math.toRadians(180));
        moveAwayFromWallBlueBackdrop = new Pose2d(-50, 15, Math.toRadians(0));

        moveAwayFromWallRedStacks = new Pose2d(52, -35, Math.toRadians(180));
        moveAwayFromWallBlueStacks = new Pose2d(-54, -35, Math.toRadians(0));

        moveTowardsRedBackdrop = new Pose2d(65.5, 20, Math.toRadians(270));
        moveTowardsBlueBackdrop = new Pose2d(-60, 8, Math.toRadians(270));

        moveTowardsRedBackdropLft = new Pose2d(59, 15, Math.toRadians(270));

        moveTowardsLoadStation = new Pose2d(59, -48, Math.toRadians(270));

        pickUpPixelStackCenterTapeBlue = new Pose2d(-38, -61, Math.toRadians(-90));
        pickUpPixelStackRightTapeBlue = new Pose2d(-35, -61, Math.toRadians(-90));
        pickUpPixelStackLeftTapeBlue = new Pose2d(-35, -61, Math.toRadians(-90));

        pickUpPixelStackRightTapeRed = new Pose2d(37.5, -62, Math.toRadians(-90));
        pickUpPixelStackLeftTapeRed = new Pose2d(36.5, -63.5, Math.toRadians(-90));
        pickUpPixelStackCenterTapeRed = new Pose2d(37.5, -58.5, Math.toRadians(-90));

        moveTowardsBlueBackdropHdAdj = new Pose2d(-35, 38, Math.toRadians(245));
        moveTowardsRedBackdropHdAdj = new Pose2d(44, 38, Math.toRadians(270));

        moveTowardsRedBackdropHdAdjLft = new Pose2d(35, 30, Math.toRadians(295));
        moveTowardsRedBackdropHdAdjRt = new Pose2d(48, 45, Math.toRadians(295));

        dropPixelRedRightTapeBackdrop = new Pose2d(34, 24, Math.toRadians(0));
        dropPixelRedCenterTapeBackdrop = new Pose2d(31, 15, Math.toRadians(0));
        dropPixelRedLeftTapeBackdrop = new Pose2d(32, 5, Math.toRadians(90));

        dropPixelRedLeftTapeBackdropAdj = new Pose2d(32, 9, Math.toRadians(90));
        dropPixelRedRightTapeBackdropAdj = new Pose2d(36, 27, Math.toRadians(0));

        dropPixelBlueRightTapeBackdrop = new Pose2d(-30, 5.5, Math.toRadians(90));
        dropPixelBlueCenterTapeBackdrop = new Pose2d(-30, 15, Math.toRadians(180));
        dropPixelBlueLeftTapeBackdrop = new Pose2d(-33, 18, Math.toRadians(270));

        dropPixelBlueCenterBackTapeAdj = new Pose2d(-34, 15, Math.toRadians(-180));
        dropPixelRedCenterBackTapeAdj = new Pose2d(32, 18, Math.toRadians(0));

        dropPixelRedRightTapeStacks = new Pose2d(31, -30.5, Math.toRadians(270));
        dropPixelRedCenterTapeStacks = new Pose2d(20.5, -45, Math.toRadians(270));
        dropPixelRedLeftTapeStacks = new Pose2d(32, -46, Math.toRadians(0));

        dropPixelRedRightTapeStacksAdj = new Pose2d(33, -35, Math.toRadians(270));

        dropPixelRedCenterStkTapeAdj = new Pose2d(26, -45, Math.toRadians(270));
        dropPixelBlueCenterStkTapeAdj = new Pose2d(-27, -46, Math.toRadians(270));

        dropPixelBlueRightTapeStacks = new Pose2d(-32, -52, Math.toRadians(270));
        dropPixelBlueCenterTapeStacks = new Pose2d(-18, -42, Math.toRadians(270));
        dropPixelBlueLeftTapeStacks = new Pose2d(-33, -30, Math.toRadians(270));

        moveFromBlueRightTapeStacks = new Pose2d(-32, -57.5, Math.toRadians(270));
        moveFromBlueLeftTapeStacks = new Pose2d(-33, -50, Math.toRadians(270));
        moveFromBlueCenterTapeStacks = new Pose2d(-24, -52, Math.toRadians(270));

        moveFromRedLeftTapeStacks = new Pose2d(59, -50, Math.toRadians(270));
        moveFromRedRightTapeStacks = new Pose2d(32, -45, Math.toRadians(270));
        moveFromRedCenterTapeStacks = new Pose2d(24, -57, Math.toRadians(230));

        dropOnBackdropBlueLeftBackdrop = new Pose2d(-42, 52.5, Math.toRadians(270));
        dropOnBackdropBlueRightBackdrop = new Pose2d(-28, 53, Math.toRadians(270));
        dropOnBackdropBlueCenterBackdrop = new Pose2d(-37, 53.5, Math.toRadians(270));

        dropOnBackdropBlueLeftBackdropHi = new Pose2d(-43, 55.5, Math.toRadians(270));
        dropOnBackdropBlueRightBackdropHi = new Pose2d(-25, 55.5, Math.toRadians(270));
        dropOnBackdropBlueCenterBackdropHi = new Pose2d(-37, 55.5, Math.toRadians(270));

        dropOnBackdropRedLeftBackdrop = new Pose2d(22.5, 52, Math.toRadians(270));
        dropOnBackdropRedRightBackdrop = new Pose2d(45.5, 56.3, Math.toRadians(270));
        dropOnBackdropRedCenterBackdrop = new Pose2d(36.5, 55.5, Math.toRadians(270));

        moveAwayFromCRedBackdropTape = new Pose2d(38, 15, Math.toRadians(0));
        moveAwayFromLRedBackdropTape = new Pose2d(32, 15, Math.toRadians(90));
        moveAwayFromRRedBackdropTape = new Pose2d(41, 24, Math.toRadians(0));

        moveAwayFromCBlueBackdropTape = new Pose2d(-45, 15, Math.toRadians(180));
        moveAwayFromLBlueBackdropTapeAdj = new Pose2d(-33, 14, Math.toRadians(270));
        moveAwayFromRBlueBackdropTape = new Pose2d(-32, 15, Math.toRadians(90));

        moveAwayFromLBlueBackdropTape = new Pose2d(-50, 14, Math.toRadians(270));

        reverseFromRedBackdropBk = new Pose2d(36, 35, Math.toRadians(270));
        reverseFromBlueBackdropBk = new Pose2d(-36, 35, Math.toRadians(270));

        reverseFromRedBackdropStk = new Pose2d(36, 42, Math.toRadians(310));
        reverseFromBlueBackdropStk = new Pose2d(-36, 42, Math.toRadians(230));

        dropOnBackdropBlueLeftStacks = new Pose2d(-37, 55.5, Math.toRadians(270));
        dropOnBackdropBlueRightStacks = new Pose2d(-15.5, 59, Math.toRadians(270));
        dropOnBackdropBlueCenterStacks = new Pose2d(-29, 57, Math.toRadians(270));

        dropOnBackdropBlueLeftStacksHi = new Pose2d(-37, 57, Math.toRadians(270));
        dropOnBackdropBlueRightStacksHi = new Pose2d(-15.5, 60, Math.toRadians(270));
        dropOnBackdropBlueCenterStacksHi = new Pose2d(-29, 58.5, Math.toRadians(270));

        dropOnBackdropRedLeftStacks = new Pose2d(27.5, 53.5, Math.toRadians(270));
        dropOnBackdropRedRightStacks = new Pose2d(49, 67, Math.toRadians(270));
        dropOnBackdropRedCenterStacks = new Pose2d(42.5, 57.5, Math.toRadians(270));

        dropOnBackdropRedLeftStacksHi = new Pose2d(27.5, 54.5, Math.toRadians(270));
        dropOnBackdropRedRightStacksHi = new Pose2d(49, 69, Math.toRadians(270));
        dropOnBackdropRedCenterStacksHi = new Pose2d(44, 58, Math.toRadians(270));

        backAwayFromRedTape = new Pose2d(65.5, -30, Math.toRadians(270));
        backAwayFromBlueTape = new Pose2d(-60, -45, Math.toRadians(270));

        parkBlueStacksSide = new Pose2d(0, 55, Math.toRadians(230));
        parkRedStacksSide = new Pose2d(12, 53, Math.toRadians(270));
        /* End Qualifier Route Points */

         if (INIT_TRAJ_2 != RobotConstants.trajType)
         {

         }
         if (INIT_TRAJ_1 != RobotConstants.trajType)
         {
             initTrajectories2();
         }

     }


    protected void initTrajectories2()
     {
         finalizeTrajSeq();
     }

    public String toString()
     {
         return "";
     }

     public final static int INIT_CONE_STACK = 5;
     public static int conestackNum = INIT_CONE_STACK;
	 protected Field.Highways parkPos;
     protected PositionOption startPos;
     protected Field.Alliance alliance;
     protected Field.Route routeStrategy;
     protected Field.FirstLocation firstLocation;
     protected TeamElement teamElement;
     protected Field.Highways[] highways;
	 protected Field.Highways[] pixelStacks;
     protected int numCycles = 0;
     protected final double botLen;
     protected final double botWid;
     protected final double botBackToCtr;
     protected final double botSideToCtr;
     protected final double hubRad;

     protected final int sx;
     protected final int sy;
     protected double sh;
     protected int sf;
     protected int sr;
     protected double flip;
     protected final double strtX;
     protected final double strtY;
     protected final double strtH;

    protected final Pose2d pickUpPixelStackRightTapeRed;
    protected final Pose2d pickUpPixelStackLeftTapeRed;
    protected final Pose2d pickUpPixelStackCenterTapeRed;
    protected final Pose2d pickUpPixelStackLeftTapeBlue;
    protected final Pose2d pickUpPixelStackCenterTapeBlue;
    protected final Pose2d pickUpPixelStackRightTapeBlue;
    protected final Pose2d parkDoor;
    protected final Pose2d parkWall;
    protected final Pose2d parkDoorAdj;
    protected final Pose2d parkWallAdj;

    protected final Pose2d parkDoorRed;
    protected final Pose2d parkWallRed;
    protected final Pose2d parkDoorAdjRed;
    protected final Pose2d parkWallAdjRed;

    protected final Pose2d parkWallRedRight;



    /* Qualifier Route Points */
    protected Pose2d moveAwayFromWallRedBackdrop;
    protected Pose2d moveAwayFromWallBlueBackdrop;

    protected Pose2d moveAwayFromWallRedStacks;
    protected Pose2d moveAwayFromWallBlueStacks;

    protected Pose2d moveTowardsRedBackdrop;
    protected Pose2d moveTowardsBlueBackdrop;

    protected Pose2d moveTowardsRedBackdropLft;
    protected Pose2d moveTowardsLoadStation;

    protected Pose2d moveTowardsBlueBackdropHdAdj;
    protected Pose2d moveTowardsRedBackdropHdAdj;

    protected Pose2d moveTowardsRedBackdropHdAdjLft;
    protected Pose2d moveTowardsRedBackdropHdAdjRt;

    protected Pose2d dropPixelRedRightTapeBackdrop;
    protected Pose2d dropPixelRedCenterTapeBackdrop;
    protected Pose2d dropPixelRedLeftTapeBackdrop;

    protected Pose2d dropPixelRedLeftTapeBackdropAdj;
    protected Pose2d dropPixelRedRightTapeBackdropAdj;

    protected Pose2d dropPixelBlueRightTapeBackdrop;
    protected Pose2d dropPixelBlueCenterTapeBackdrop;
    protected Pose2d dropPixelBlueLeftTapeBackdrop;

    protected Pose2d dropPixelRedRightTapeStacks;
    protected Pose2d dropPixelRedCenterTapeStacks;

    protected Pose2d dropPixelRedRightTapeStacksAdj;

    protected Pose2d dropPixelRedCenterStkTapeAdj;
    protected Pose2d dropPixelBlueCenterStkTapeAdj;

    protected Pose2d dropPixelRedCenterBackTapeAdj;
    protected Pose2d dropPixelRedLeftBackTapeAdj;
    protected Pose2d dropPixelBlueCenterBackTapeAdj;

    protected Pose2d dropPixelBlueCenterTapeStacks;
    protected Pose2d dropPixelBlueLeftTapeStacks;

    protected Pose2d moveFromBlueRightTapeStacks;
    protected Pose2d moveFromBlueLeftTapeStacks;
    protected Pose2d moveFromBlueCenterTapeStacks;

    protected Pose2d moveFromRedRightTapeStacks;
    protected Pose2d moveFromRedLeftTapeStacks;
    protected Pose2d moveFromRedCenterTapeStacks;

    protected Pose2d dropOnBackdropBlueLeftBackdrop;
    protected Pose2d dropOnBackdropBlueRightBackdrop;
    protected Pose2d dropOnBackdropBlueCenterBackdrop;

    protected Pose2d dropOnBackdropBlueLeftBackdropHi;
    protected Pose2d dropOnBackdropBlueRightBackdropHi;
    protected Pose2d dropOnBackdropBlueCenterBackdropHi;

    protected Pose2d dropOnBackdropRedLeftBackdrop;
    protected Pose2d dropOnBackdropRedRightBackdrop;
    protected Pose2d dropOnBackdropRedCenterBackdrop;

    protected Pose2d moveAwayFromCRedBackdropTape;
    protected Pose2d moveAwayFromLRedBackdropTape;
    protected Pose2d moveAwayFromRRedBackdropTape;

    protected Pose2d moveAwayFromCBlueBackdropTape;
    protected Pose2d moveAwayFromLBlueBackdropTapeAdj;
    protected Pose2d moveAwayFromRBlueBackdropTape;

    protected Pose2d moveAwayFromLBlueBackdropTape;

    protected Pose2d reverseFromRedBackdropBk;
    protected Pose2d reverseFromBlueBackdropBk;

    protected Pose2d reverseFromRedBackdropStk;
    protected Pose2d reverseFromBlueBackdropStk;

    protected Pose2d dropOnBackdropBlueLeftStacks;
    protected Pose2d dropOnBackdropBlueRightStacks;
    protected Pose2d dropOnBackdropBlueCenterStacks;

    protected Pose2d dropOnBackdropBlueLeftStacksHi;
    protected Pose2d dropOnBackdropBlueRightStacksHi;
    protected Pose2d dropOnBackdropBlueCenterStacksHi;

    protected Pose2d dropOnBackdropRedLeftStacks;
    protected Pose2d dropOnBackdropRedRightStacks;
    protected Pose2d dropOnBackdropRedCenterStacks;

    protected Pose2d dropOnBackdropRedLeftStacksHi;
    protected Pose2d dropOnBackdropRedRightStacksHi;
    protected Pose2d dropOnBackdropRedCenterStacksHi;

    protected Pose2d backAwayFromRedTape;
    protected Pose2d backAwayFromBlueTape;

    protected Pose2d parkBlueBackdropSide;
    protected Pose2d parkRedBackdropSide;
    protected Pose2d parkBlueStacksSide;
    protected Pose2d parkRedStacksSide;

  protected Pose2d starting_point_top;

    protected Pose2d dropPixelRedLeftTapeStacks;
    protected Pose2d dropPixelBlueRightTapeStacks;

    /* End Qualifier Route Points */




    /* Start */
    public final Pose2d start;

	 /* Map for new route, leaves old route in tact in case we need to revert back to it to validate */
     TrajectorySequence fullSeq;

     /* New way to define map */
     public List<TrajectorySequence> trajList;
     public EnumMap<Field.ParkPos,TrajectorySequence> parkMap = new EnumMap<>(Field.ParkPos.class);;
     private Pose2d lastPose;
     private Pose2d firstPose;
     private TrajectorySequenceBuilder traj;
     protected double totalDur;



     public enum Movement
     {
          START, //used for first position, i.e. no moving to this location
          LINE,
          STRAFE,
          STRAFE_LEFT,
          STRAFE_RIGHT,
          SPLINE,
          FORWARD,
          BACK,
          TURN
     }
     public enum Heading
     {
          HEAD_DEFAULT,
          HEAD_CONSTANT,
          HEAD_LINEAR,
          HEAD_SPLINE
     }

     public enum Action
     {
          WAIT,
          TANGENT
     }

     public enum TeamElement
     {
         LEFT,
         RIGHT,
         CENTER
     }

   public double calcTimeOrSomething(){
       double cumTime = 0.0;

       if(null != trajList)
       {


           int seqNum = 0;



           for (TrajectorySequence tSeq : trajList)
           {
               String seqName = String.valueOf(seqNum++);
               for (int i = 0; i < tSeq.size(); i++)
               {
                   SequenceSegment seg = tSeq.get(i);

                   if (seg instanceof TrajectorySegment)
                   {
                       Trajectory t = ((TrajectorySegment) seg).getTrajectory();
                       cumTime += t.duration();
                   }
                   else
                   {
                       if (seg instanceof TurnSegment)
                       {
                           TurnSegment ts = (TurnSegment) seg;
                           cumTime += ts.getDuration();
                       }
                       else if (seg instanceof WaitSegment)
                       {
                           WaitSegment ws = (WaitSegment) seg;
                           cumTime += ws.getDuration();
                       }
                   }
               }
           }

       }
       return cumTime;
    }
     boolean justStarted = true;

     public void addLocation(Pose2d loc, Movement move, Heading head)
     {
         addLocation(loc, move, head, false, 0);
     }

    public void addLocation(Pose2d loc, Movement move, Heading head, double customHeadValue)
    {
        addLocation(loc, move, head, true, customHeadValue);
    }

    private boolean newTraj = false;

    private void makeNewTraj()
    {
         if(null != traj)
         {
           trajList.add(traj.build());
         }
         traj = new TrajectorySequenceBuilder(
                 lastPose, null,
                 defVelLim, defAccelLim,
                 RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
    }

    public void addParkPosition(Field.ParkPos parkEnum, Pose2d loc, Movement move, Heading head)
    {
         Pose2d origLastPose = lastPose;
         addLocation(loc, move, head, false, 0);
         parkMap.put(parkEnum, traj.build());
         try
  	     {
             System.out.printf(Locale.US, "putting park position, %d\"", parkEnum);
         }
  	     catch (Exception e)
  	     {
             System.out.println("failure putting park position in log");
         }
         traj = null;
         lastPose = origLastPose;
    }

    private boolean samePose(Pose2d loc){
        boolean sp = false;
        if(loc.getY() == lastPose.getY() && loc.getX() == lastPose.getX()){
            sp = true;
            if(loc.getHeading() != lastPose.getHeading()){
                //TODO: throw compiler warning
            }
        }
        return sp;
    }

    public void addLocation(Pose2d loc, Movement move, Heading head, boolean custom, double customHeadValue)
    {
        if(null != lastPose)
  	    {
              if(!samePose(loc)) {
                  if (justStarted) {
                      justStarted = false;
                      trajList = new ArrayList<TrajectorySequence>();
                  }
                  if (newTraj) {
                      makeNewTraj();
                  } else {
                      newTraj = false;
                      //when not in debug, default to false unless something set it to true;
                  }

             switch(move)
  	         {
                  case START:
                      //acts as a restart; should not normally be used
                      firstPose = loc;
                      justStarted = true;
                      traj = new TrajectorySequenceBuilder(
                              firstPose,
                              defVelLim, defAccelLim,
                              RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
                      break;
                  case LINE:
                      switch (head)
                      {
                         case HEAD_LINEAR:
                           traj.lineToLinearHeading(loc);
                           break;
                         case HEAD_CONSTANT:
                           traj.lineToConstantHeading(loc.vec());
                           break;
                         case HEAD_SPLINE:
                           traj.lineToSplineHeading(loc);
                           break;
                         case HEAD_DEFAULT:
                         default:
                           traj.lineTo(loc.vec());
                           break;
                      }
                      break;
                  case STRAFE:
                      traj.strafeTo(loc.vec());
                      break;
                  default: //Spline or anything else
                      double headValue;

                      if(custom)
                      {
                          headValue = customHeadValue;
                      }
                      else
                      {
                          headValue = loc.getHeading();
                      }
                      switch (head)
                      {
                            case HEAD_LINEAR:
                              traj.splineToLinearHeading(loc,headValue);
                              break;
                            case HEAD_SPLINE:
                              traj.splineToSplineHeading(loc,headValue + flip);
                              break;
                            case HEAD_CONSTANT:
                              traj.splineToConstantHeading(loc.vec(),headValue + flip);
                              break;
                            case HEAD_DEFAULT:
                            default:
                              traj.splineTo(loc.vec(),headValue);
                              break;
                      }
                      break;
                  }
             }
        }
        else
  	    {
  	        //What should happen when first called with START
            firstPose = loc;
            justStarted = true;
            traj = new TrajectorySequenceBuilder(
                    firstPose,
                    defVelLim, defAccelLim,
                    RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
        }

        lastPose = loc; //Setup for next time

     }

     public void addMovement(Movement move, Double amount)
     {
          switch (move)
  	      {
               case STRAFE_RIGHT:
                 traj.strafeRight(amount);
                 break;
               case STRAFE_LEFT:
                 traj.strafeLeft(amount);
                 break;
               case FORWARD:
                 traj.forward(amount);
                 break;
               case BACK:
                 traj.back(amount);
                 break;
               case TURN:
                 traj.turn(amount);
                 break;
          }
     }

     public void armDrivePos()
     {

     }
     public void armDropSpikePos(){
}
    public void outFrontPixel(){
        Timer timer = new Timer();
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                armDropSpikePos();
            }
        };
        timer.schedule(task, 2000);
    }
     public void armToDrop(){
     }

    public void armToDropHigher(){

    }
    public void armToIntake(){

    }
     public void outPixel(){
     }
    public void outPurplePixel(){
     }
     public void intakes(){
     }

     public void addFunction(MarkerCallback callback)
     {
          traj.addTemporalMarker(callback);
     }

     public void addFunction(MarkerCallback callback, double time)
     {
          traj.addTemporalMarker(time, callback);
     }

     public void addEvent(Action act, double value)
     {
          switch(act)
  	      {
              case WAIT:
                  traj.waitSeconds(value);
                  break;
              case TANGENT:
                  try{          //cath error when trying to add tangent to first trajectory. TODO: make this not needed
                      makeNewTraj();
                  }catch(Exception e){

                  }
                  newTraj = false;
                  traj.setTangent(value);
                  break;
          }
     }

    public void addTrajectory(TrajectorySequence trajectory)
    {
         if(null == trajList)
  	     {
             addLocation(trajectory.start(), Movement.START, Heading.HEAD_DEFAULT);
         }
         if(lastPose != trajectory.start())
  	     {
             addLocation(trajectory.start(), Movement.SPLINE, Heading.HEAD_SPLINE);
         }
         trajList.add(trajectory);
         lastPose = trajectory.end();
         newTraj = true;
    }

    public void addParkTrajectory(Field.ParkPos parkEnum, TrajectorySequence trajectory)
    {
         parkMap.put(parkEnum, trajectory);
    }

    public void finalizeTrajSeq()
    {
        /* Finalize Traj Seq */
        if(null != trajList)
	    {
             if(null != traj)
	         {
                 trajList.add(traj.build());
             }
             if(null != fullSeq)
	         {
                 addTrajectory(fullSeq);
             }

             TrajectorySequenceBuilder full = new TrajectorySequenceBuilder(
                     firstPose,
                     defVelLim, defAccelLim,
                     RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);

             double cumTime = 0.0;
             System.out.printf(Locale.US, "BOT:%s ALLIANCE:%s START:%s PARK:%s%n",
                     RobotConstants.bot, alliance, startPos, parkPos);

             int seqNum = 0;
             TrajectorySequence parkTraj = parkMap.get(parkPos);
             if(null != parkTraj) {
                 trajList.add(parkTraj);
             }

             for (TrajectorySequence tSeq : trajList)
             {
                  String seqName = String.valueOf(seqNum++);
                  //TrajectorySequence tSeq = e.getValue();
                  System.out.printf(Locale.US, "Trajectory Sequence %s len=%d dur=%.2f%n",
                          seqName, tSeq.size(), tSeq.duration());
                  for (int i = 0; i < tSeq.size(); i++)
                  {
                       SequenceSegment seg = tSeq.get(i);

                       if (seg instanceof TrajectorySegment)
                       {
                            Trajectory t = ((TrajectorySegment) seg).getTrajectory();
                            full.addTrajectory(t);
                            System.out.printf(Locale.US, "  TrajSeg %d %s to %s in %.2f at %.2f%n",
                                    i, seg.getStartPose(), seg.getEndPose(), seg.getDuration(), cumTime);
                            for (TrajectoryMarker m : ((TrajectorySegment) seg).getTrajectory().getMarkers())
                            {
                              System.out.printf(Locale.US, "    marker in Traj @ %.2f%n", m.getTime());
                            }
                            cumTime += t.duration();
                       }
                       else
                       {
                            if (seg instanceof TurnSegment)
                            {
                                 TurnSegment ts = (TurnSegment) seg;
                                 full.turn(ts.getTotalRotation());
                                 System.out.printf(Locale.US, "  TurnSeg %d %s to %s in %.2f at %.2f%n",
                                         i, seg.getStartPose(), seg.getEndPose(), seg.getDuration(), cumTime);
                                 for (TrajectoryMarker m : seg.getMarkers())
                                 {
                                      full.addTemporalMarker(m.getTime(), m.getCallback());
                                      System.out.printf(Locale.US, "    marker in Turn @ %.2f%n", m.getTime());
                                 }
                                 cumTime += ts.getDuration();
                            }
                            else if (seg instanceof WaitSegment)
                            {
                                 WaitSegment ws = (WaitSegment) seg;
                                 full.waitSeconds(ws.getDuration());
                                 System.out.printf(Locale.US, "  WaitSeg %d %s to %s in %.2f at %.2f%n",
                                                    i, seg.getStartPose(), seg.getEndPose(), seg.getDuration(), cumTime);
                                 for (TrajectoryMarker m : seg.getMarkers())
                                 {
                                     full.addTemporalMarker(m.getTime(), m.getCallback());
                                     System.out.printf(Locale.US, "    marker in Wait @ %.2f%n", m.getTime());
                                 }
                                 cumTime += ws.getDuration();
                            }
                       }
                  }
             }
         fullSeq = full.build();
         }
    }

    public enum SegmentState
    {
        MOVE_FROM_START,
		/* LEFT SIDE Trajectory Segments */
        DROP_CONE_3PT_JUNCTION,
        MOVE_FROM_3PT_JUNCTION,
        CONE_GRAB,
		/* RIGHT SIDE Trajectory Segments */
		MOVE_FROM_SHARED_5PT_JUNCTION,
		CONE_GRAB_FROM_SHARED_5PT_JUNCTION,
		/* END Of Specific Segments */		
        DROP_CONE_4PT_JUNCTION,
        REVERSE_FROM_4PT_JUNCTION,
        DROP_CONE_4PT_JUNCTION_REPEAT,
        REVERSE_FROM_4PT_JUNCTION_REPEAT,
        DROP_CONE_OPP_5PT_JUNCTION,
        PARK_RIGHT,
        PARK_LEFT,
        PARK_CENTER
    }
}