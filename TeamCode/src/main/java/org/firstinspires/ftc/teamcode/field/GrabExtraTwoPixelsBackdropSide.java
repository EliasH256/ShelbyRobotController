package org.firstinspires.ftc.teamcode.field;


import static org.firstinspires.ftc.teamcode.field.Field.ParkLocation.DOORSIDE_PARK;
import static org.firstinspires.ftc.teamcode.field.Field.ParkLocation.WALLSIDE_PARK;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.HEAD_LINEAR;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.LINE;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.TURN;

public class GrabExtraTwoPixelsBackdropSide {
    Route route;
    public GrabExtraTwoPixelsBackdropSide(Route constructorRoute) {
        route = constructorRoute;
   }

    public void makeTraj(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, PositionOption autonStrategy, Field.stacksSideExtraPixelGrab extraPixelGrab)
    {
        if (alliance == Field.Alliance.BLUE)
        {
            route.addFunction(route::armDropSpikePos);
//            route.addMovement(TURN, -0.5);
            route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
            route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackCenterTapeBlue, LINE, HEAD_LINEAR);
            route.addEvent(Route.Action.WAIT, 0.1);
            route.addFunction(route::allIntakesOn);
            route.addEvent(Route.Action.WAIT, 0.60);
            route.addFunction(route::armDropSpikePos);
            route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
            route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
        }
        else if (alliance == Field.Alliance.RED)
        {
            route.addEvent(Route.Action.WAIT, 60);
        }

    }
}


