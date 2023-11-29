package com.example.meepmeeptesting;
public class CenterStageRoute extends Route
{
  public CenterStageRoute(PositionOption startPos,
                          Field.Highways parkPos,
                          Field.Alliance alliance,
                          Field.Route autonStrat,
                          Field.FirstLocation firstLocation,
                          Route.TeamElement teamElement,
                          Field.Highways[] highways,
                          Field.Highways[] pixelStacks)
  {
      super(startPos, parkPos, alliance, autonStrat, firstLocation, teamElement, highways, pixelStacks);
  }

  /*
  Use the builder functions in Route to build routes in initTrajectories2
  common examples are:
        addLocation(Pose2d, Movement, Heading) - to move the robot to that location
        addFunction(functionName) - to perform a function
        addEvent(WAIT, waitTime) - to add a wait period
        addParkPosition(ParkPos, Pose2d, Movement, Heading) - to define where to park and how to move there
  */
    public void dropPixel(){
        //TODO: add code to make this work
    }

    public void dropPixels(){
        dropPixel();
        dropPixel();
    }

    public void pickUpPixel(){
        //TODO: add code to make this work
    }

    public void pickUpPixels(){
        pickUpPixel();
        pickUpPixel();
    }


   protected void initTrajectories2()
   {
       DropOnTapeThenBackdrop t1 = new DropOnTapeThenBackdrop(this);
       t1.makeTraj(teamElement, alliance, startPos, firstLocation);

       MoveToPark t5 = new MoveToPark(this);
       t5.makeTraj(parkPos, alliance);

        //Always do this at the end of initTrajectories2
        finalizeTrajSeq();
    }
}
