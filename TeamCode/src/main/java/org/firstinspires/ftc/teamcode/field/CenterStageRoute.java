package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.robot.MecanumBot;

public class CenterStageRoute extends Route
{
  private static final String TAG = "SJH_PPR";

  public CenterStageRoute(MecanumBot robot,
                          TeamElement teamElement,
                          PositionOption startPos,
						  Field.ParkLocation parkPos,
                          Field.Alliance alliance,
                          Field.stacksSideExtraPixelGrab firstLocation,
                          Field.PathWayToFrontStage routeToFrontStage,
                          Field.PathWayToBackStage  routeToBackStage,
                          Field.PixelStackLoc pixelPickupLoc)
  {
    super(robot, teamElement, startPos, parkPos, alliance, firstLocation, routeToFrontStage, routeToBackStage, pixelPickupLoc);
  }

  /*
  Use the builder functions in Route to build routes in initTrajectories2
  common examples are:
        addLocation(Pose2d, Movement, Heading) - to move the robot to that location
        addFunction(functionName) - to perform a function
        addEvent(WAIT, waitTime) - to add a wait period
        addParkPosition(ParkPos, Pose2d, Movement, Heading) - to define where to park and how to move there
  */
   protected void initTrajectories2()
   {
       DropOnTapeThenBackdrop t1 = new DropOnTapeThenBackdrop(this);
       t1.makeTraj(teamElement, alliance, startPos, extraPixelGrab, routeToBackStage);
       if (startPos == Field.StartPos.START_BACKDROP && extraPixelGrab == Field.stacksSideExtraPixelGrab.GRAB_EXTRA_PIXEL)
       {
           GrabExtraTwoPixelsBackdropSide t2 = new GrabExtraTwoPixelsBackdropSide(this);
           t2.makeTraj(teamElement, alliance, startPos, extraPixelGrab);
       }
       MoveToPark t4 = new MoveToPark(this);
       t4.makeTraj(parkPos, startPos, alliance, extraPixelGrab, teamElement);

       /* Always do this at the end of initTrajectories2 */
       finalizeTrajSeq();
    }
}
