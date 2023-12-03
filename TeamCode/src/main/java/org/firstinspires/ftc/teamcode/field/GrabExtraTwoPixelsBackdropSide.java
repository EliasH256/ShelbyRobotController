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

    public void makeTraj(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, Field.stacksSideExtraPixelGrab extraPixelGrab)
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
                route.addEvent(Route.Action.WAIT, 0.65);
                route.addFunction(route::outFrontPixel);
                route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addFunction(route::armToDropHigher);
                route.addLocation(route.dropOnBackdropBlueCenterTwoPixels, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::outTwoPixels);
                route.addEvent(Route.Action.WAIT, 0.6);
            }
            if (teamElement == Route.TeamElement.RIGHT)
            {
                route.addMovement(TURN, -0.5);
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.blueTwoPixelBackstageRt, LINE, HEAD_LINEAR);
                route.addLocation(route.blueTwoPixelStacksRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armToIntake);
                route.addLocation(route.pickUpPixelStackCenterTapeBlueBackdropAdj, LINE, HEAD_LINEAR);
                route.addFunction(route::allIntakesOn);
                route.addLocation(route.pickUpPixelStackCenterTapeBlueBackdrop, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.65);
                route.addFunction(route::outFrontPixel);
                route.addLocation(route.blueTwoPixelStacksRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.blueTwoPixelBackstageRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armToDropHigher);
                route.addLocation(route.dropOnBackdropBlueRightTwoPixels, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::outTwoPixels);
                route.addEvent(Route.Action.WAIT, 0.6);
            }
            if (teamElement == Route.TeamElement.LEFT)
            {
                route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armToIntake);
                route.addLocation(route.pickUpPixelStackLeftTapeBlueBackdropAdj, LINE, HEAD_LINEAR);
                route.addFunction(route::allIntakesOn);
                route.addLocation(route.pickUpPixelStackLeftTapeBlueBackdrop, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.65);
                route.addFunction(route::outFrontPixel);
                route.addLocation(route.blueTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.blueTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addFunction(route::armToDropHigher);
                route.addLocation(route.dropOnBackdropBlueLeftTwoPixels, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::outTwoPixels);
                route.addEvent(Route.Action.WAIT, 0.6);
            }
        }
        else if (alliance == Field.Alliance.RED)
        {
            if (teamElement == Route.TeamElement.CENTER)
            {
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.redTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addLocation(route.redTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armToIntake);
                route.addLocation(route.pickUpPixelStackCenterTapeRedBackdropAdj, LINE, HEAD_LINEAR);
                route.addFunction(route::allIntakesOn);
                route.addLocation(route.pickUpPixelStackCenterTapeRedBackdrop, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.65);
                route.addFunction(route::outFrontPixel);
                route.addLocation(route.redTwoPixelStacks, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.redTwoPixelBackstage, LINE, HEAD_LINEAR);
                route.addFunction(route::armToDropHigher);
                route.addLocation(route.dropOnBackdropRedCenterTwoPixels, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::outTwoPixels);
                route.addEvent(Route.Action.WAIT, 0.6);
            }
            if (teamElement == Route.TeamElement.RIGHT)
            {
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.redTwoPixelBackstageRt, LINE, HEAD_LINEAR);
                route.addLocation(route.redTwoPixelStacksRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armToIntake);
                route.addLocation(route.pickUpPixelStackRightTapeRedBackdropAdj, LINE, HEAD_LINEAR);
                route.addFunction(route::allIntakesOn);
                route.addLocation(route.pickUpPixelStackRightTapeRedBackdrop, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.65);
                route.addFunction(route::outFrontPixel);
                route.addLocation(route.redTwoPixelStacksRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.redTwoPixelBackstageRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armToDropHigher);
                route.addLocation(route.dropOnBackdropRedRightTwoPixels, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::outTwoPixels);
                route.addEvent(Route.Action.WAIT, 0.6);
            }
            if (teamElement == Route.TeamElement.LEFT)
            {
                route.addLocation(route.redTwoPixelBackstageRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.redTwoPixelStacksRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armToIntake);
                route.addLocation(route.pickUpPixelStackLeftTapeRedBackdropAdj, LINE, HEAD_LINEAR);
                route.addFunction(route::allIntakesOn);
                route.addLocation(route.pickUpPixelStackLeftTapeRedBackdrop, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.55);
                route.addFunction(route::outFrontPixel);
                route.addLocation(route.redTwoPixelStacksRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armDropSpikePos);
                route.addLocation(route.redTwoPixelBackstageRt, LINE, HEAD_LINEAR);
                route.addFunction(route::armToDropHigher);
                route.addLocation(route.dropOnBackdropRedLeftTwoPixels, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::outTwoPixels);
                route.addEvent(Route.Action.WAIT, 0.6);
            }
        }
    }
}


