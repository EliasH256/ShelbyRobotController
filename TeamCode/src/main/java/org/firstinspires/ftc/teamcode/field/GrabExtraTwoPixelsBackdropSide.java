package org.firstinspires.ftc.teamcode.field;


import static org.firstinspires.ftc.teamcode.field.Field.ParkLocation.DOORSIDE_PARK;
import static org.firstinspires.ftc.teamcode.field.Field.ParkLocation.WALLSIDE_PARK;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.HEAD_LINEAR;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.LINE;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.SPLINE;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.TURN;

public class GrabExtraTwoPixelsBackdropSide
{
    Route route;
    public GrabExtraTwoPixelsBackdropSide(Route constructorRoute)
    {
        route = constructorRoute;
    }

    public void makeTraj(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, PositionOption autonStrategy, Field.stacksSideExtraPixelGrab extraPixelGrab)
    {
        if (alliance == Field.Alliance.BLUE)
        {
            if (teamElement == Route.TeamElement.CENTER)
            {
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armToIntake);
                route.addLocation(route.pickUpPixelStackCenterTapeBlueBackdropAdj, LINE, HEAD_LINEAR);
                route.addFunction(route::allIntakesOn);
                route.addLocation(route.pickUpPixelStackCenterTapeBlueBackdrop, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.55);
                route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addFunction(route::outFrontPixel);
                route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addFunction(route::armToDropHigher);
                route.addLocation(route.dropOnBackdropBlueTwoPixels, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::outPixel);
                route.addEvent(Route.Action.WAIT, 0.5);
            }
            if (teamElement == Route.TeamElement.RIGHT)
            {
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armToIntake);
                route.addLocation(route.pickUpPixelStackCenterTapeBlueBackdropAdj, LINE, HEAD_LINEAR);
                route.addFunction(route::allIntakesOn);
                route.addLocation(route.pickUpPixelStackCenterTapeBlueBackdrop, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.55);
                route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addFunction(route::outFrontPixel);
                route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addFunction(route::armToDropHigher);
                route.addLocation(route.dropOnBackdropBlueTwoPixels, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::outPixel);
                route.addEvent(Route.Action.WAIT, 0.5);
            }
            if (teamElement == Route.TeamElement.LEFT)
            {
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armToIntake);
                route.addLocation(route.pickUpPixelStackCenterTapeBlueBackdropAdj, LINE, HEAD_LINEAR);
                route.addFunction(route::allIntakesOn);
                route.addLocation(route.pickUpPixelStackCenterTapeBlueBackdrop, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.55);
                route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addFunction(route::outFrontPixel);
                route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addFunction(route::armToDropHigher);
                route.addLocation(route.dropOnBackdropBlueTwoPixels, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::outPixel);
                route.addEvent(Route.Action.WAIT, 0.5);
            }
        }
        else if (alliance == Field.Alliance.RED)
        {
            route.addEvent(Route.Action.WAIT, 60);
        }

    }
}


