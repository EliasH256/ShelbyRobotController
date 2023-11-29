package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Field.stacksSideExtraPixelGrab.*;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;

public class DropOnTapeThenBackdrop {
    Route route;
    public DropOnTapeThenBackdrop(Route constructorRoute) {
route = constructorRoute;
    }

    public void makeTraj(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, PositionOption autonStrategy, Field.stacksSideExtraPixelGrab extraPixelGrab)
    {
        if (autonStrategy == Field.Route.QUALIFIER_ROUTE)
        {
            /* Option to run the routes used in Qualifiers in case of a slow alliance partner
               Should use Drive Constants that were tuned for the Qualifiers */
            qualifierRoute(teamElement, alliance, startPos, extraPixelGrab);
        }
        else if (autonStrategy == Field.Route.STATES_ROUTE && extraPixelGrab == GRAB_EXTRA_PIXEL)
        {
            statesRoute(teamElement, alliance, startPos, extraPixelGrab);
        }
        else
        {
            qualifierRoute(teamElement, alliance, startPos, extraPixelGrab);
        }
    }

    private void qualifierRoute(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, Field.stacksSideExtraPixelGrab extraPixelGrab)
    {
        if (alliance == Field.Alliance.RED)
        {
            switch ((Field.StartPos)startPos)
            {
                /* Blue Alliance */
                case START_BACKDROP:
                    route.addLocation(route.start, SPLINE, HEAD_LINEAR);
                    switch (teamElement)
                    {
                        case LEFT:
                            // Red Left Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedLeftTapeBackdropAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedLeftTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromLRedBackdropTape, LINE, HEAD_LINEAR);
                            route.addMovement(TURN, -0.9);
                            route.addEvent(Route.Action.WAIT, 0.3);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.1);
                            route.addMovement(TURN, 0.7);
                            route.addEvent(Route.Action.WAIT, 0.3);
                            route.addLocation(route.dropOnBackdropRedLeftBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            break;
                        case CENTER:
                            // Red Center Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addLocation(route.dropPixelRedCenterBackTapeAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedCenterTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromCRedBackdropTape, LINE, HEAD_LINEAR);
                            route.addMovement(TURN, -0.9);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropRedCenterBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);

                            break;
                        case RIGHT:
                            // Red Right Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedRightTapeBackdropAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedRightTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromRRedBackdropTape, LINE, HEAD_LINEAR);
                            route.addMovement(TURN, -0.9);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropRedRightBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            break;
                    }
                    break;
                case START_STACKS:
                    route.addLocation(route.start, START, HEAD_LINEAR);
                    switch (teamElement)
                    {
                        case LEFT:
                            // Red Left Stacks (7252)
                            route.addLocation(route.moveAwayFromWallRedStacks, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.3);
                            if (extraPixelGrab == NO_EXTRA_PIXEL)
							{
                            route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
                            }
						    else if(extraPixelGrab == GRAB_EXTRA_PIXEL)
							{
                                route.addLocation(route.pickUpPixelStackLeftTapeRed, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, .45);
                                route.addFunction(route::armDropSpikePos,1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                            }
                            route.addLocation(route.moveTowardsLoadStation,LINE, HEAD_LINEAR);
                            route.addLocation(route.moveTowardsRedBackdropLft,LINE, HEAD_LINEAR);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.moveTowardsRedBackdropHdAdjLft,LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropRedLeftStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDropHigher);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addFunction(route::outPixel);
                            route.addLocation(route.dropOnBackdropRedLeftStacksHi, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.moveTowardsRedBackdropHdAdj,LINE, HEAD_LINEAR);
                            break;
                        case CENTER:
                            // Red Center Stacks (7252)
                            route.addLocation(route.moveAwayFromWallRedStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.dropPixelRedCenterStkTapeAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedCenterTapeStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            if (extraPixelGrab == NO_EXTRA_PIXEL)
							{
                                route.addLocation(route.moveFromRedCenterTapeStacks, LINE, HEAD_LINEAR);
                            }
							else if(extraPixelGrab == GRAB_EXTRA_PIXEL)
							{
                                route.addLocation(route.pickUpPixelStackCenterTapeRed, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, .45);
                                route.addFunction(route::armDropSpikePos,1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);

                            }
                            route.addLocation(route.moveTowardsLoadStation,LINE, HEAD_LINEAR);
                            route.addLocation(route.backAwayFromRedTape, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveTowardsRedBackdrop,LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.moveTowardsRedBackdropHdAdj,LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropRedCenterStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDropHigher);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.dropOnBackdropRedCenterStacksHi, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.moveTowardsRedBackdropHdAdj,LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            break;
                        case RIGHT:
                            // Red Right Stacks (7252)
                            route.addLocation(route.moveAwayFromWallRedStacks, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedRightTapeStacksAdj, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addLocation(route.dropPixelRedRightTapeStacks, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            if (extraPixelGrab == NO_EXTRA_PIXEL)
                            {
                                route.addLocation(route.moveFromRedRightTapeStacks, LINE, HEAD_LINEAR);
                            }
                            else if(extraPixelGrab == GRAB_EXTRA_PIXEL)
                            {
                                route.addLocation(route.pickUpPixelStackRightTapeRed, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, .45);
                                route.addFunction(route::armDropSpikePos,1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                            }
                            route.addLocation(route.moveTowardsLoadStation,LINE, HEAD_LINEAR);
                            route.addLocation(route.backAwayFromRedTape, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveTowardsRedBackdrop,LINE, HEAD_LINEAR);
                            route.addFunction(route::armToDrop);
                            route.addMovement(TURN, 0.5);
                            route.addLocation(route.moveTowardsRedBackdropHdAdjRt,LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropRedRightStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDropHigher);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.dropOnBackdropRedRightStacksHi, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.moveTowardsRedBackdropHdAdjRt,LINE, HEAD_LINEAR);
                            break;
                    }
            }
        }
        else
        {
            /* Blue Alliance */
            switch ((Field.StartPos)startPos)
            {
                case START_BACKDROP:
                    route.addLocation(route.start, SPLINE, HEAD_LINEAR);
                    switch (teamElement)
                    {
                        case LEFT:
                            // Blue Left Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallBlueBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueLeftTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromLBlueBackdropTapeAdj, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromLBlueBackdropTape, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.dropOnBackdropBlueLeftBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromBlueBackdropBk, LINE, HEAD_LINEAR);
                            break;
                        case CENTER:
                            // Blue Center Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallBlueBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueCenterBackTapeAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueCenterTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromCBlueBackdropTape, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropBlueCenterBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromBlueBackdropBk, LINE, HEAD_LINEAR);
                            break;
                        case RIGHT:
                            // Blue Right Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallBlueBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueRightTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromRBlueBackdropTape, LINE, HEAD_LINEAR);
                            route.addMovement(TURN, 2.5);
                            route.addEvent(Route.Action.WAIT, 0.3);
                            route.addLocation(route.dropOnBackdropBlueRightBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromBlueBackdropBk, LINE, HEAD_LINEAR);
                            break;
                    }
                    break;
                case START_STACKS:
                    route.addLocation(route.start, START, HEAD_LINEAR);
                    switch (teamElement)
                    {
                        case LEFT:
                            // Blue Left Stacks (7252)
                            route.addLocation(route.moveAwayFromWallBlueStacks, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.dropPixelBlueLeftTapeStacks, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            if (extraPixelGrab == NO_EXTRA_PIXEL)
							{
                                route.addLocation(route.moveFromBlueLeftTapeStacks, LINE, HEAD_LINEAR);
                            }
							else if(extraPixelGrab == GRAB_EXTRA_PIXEL)
							{
                                route.addLocation(route.pickUpPixelStackLeftTapeBlue, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                                route.addFunction(route::armDropSpikePos, .5);
                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            }
                            route.addLocation(route.backAwayFromBlueTape, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveTowardsBlueBackdrop,LINE, HEAD_LINEAR);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.moveTowardsBlueBackdropHdAdj,LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropBlueLeftStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDropHigher);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.dropOnBackdropBlueLeftStacksHi, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.reverseFromBlueBackdropStk, LINE, HEAD_LINEAR);
                            break;
                        case CENTER:
                            // Blue Center Stacks (7252)
                            route.addLocation(route.moveAwayFromWallBlueStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.dropPixelBlueCenterStkTapeAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueCenterTapeStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            if (extraPixelGrab == NO_EXTRA_PIXEL)
                            {
                                route.addLocation(route.moveFromBlueCenterTapeStacks, LINE, HEAD_LINEAR);
                            }
                            else if(extraPixelGrab == GRAB_EXTRA_PIXEL)
                            {
                                route.addLocation(route.pickUpPixelStackCenterTapeBlue, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, .45);
                                route.addFunction(route::armDropSpikePos, 1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            }
                            route.addLocation(route.backAwayFromBlueTape, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveTowardsBlueBackdrop,LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.moveTowardsBlueBackdropHdAdj,LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropBlueCenterStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDropHigher);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.dropOnBackdropBlueCenterStacksHi, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveTowardsBlueBackdropHdAdj,LINE, HEAD_LINEAR);
                            break;
                        case RIGHT:
                            // Blue Right Stacks (7252)
                            route.addLocation(route.moveAwayFromWallBlueStacks, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            if (extraPixelGrab == NO_EXTRA_PIXEL)
							{
                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            }
							else if(extraPixelGrab == GRAB_EXTRA_PIXEL)
							{
                                route.addLocation(route.pickUpPixelStackRightTapeBlue, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::armDropSpikePos,1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            }
                            route.addLocation(route.backAwayFromBlueTape, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveTowardsBlueBackdrop,LINE, HEAD_LINEAR);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.moveTowardsBlueBackdropHdAdj,LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropBlueRightStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDropHigher);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.dropOnBackdropBlueRightStacksHi, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.3);
                            route.addLocation(route.reverseFromBlueBackdropStk, LINE, HEAD_LINEAR);
                            break;
                    }
            }
        }
    }
    private void statesRoute(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, Field.stacksSideExtraPixelGrab extraPixelGrab)
    {
        if (alliance == Field.Alliance.RED)
        {
            switch ((Field.StartPos)startPos)
            {
                /* Blue Alliance */
                case START_BACKDROP:
                    route.addLocation(route.start, SPLINE, HEAD_LINEAR);
                    switch (teamElement)
                    {
                        case LEFT:
                            // Red Left Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedLeftTapeBackdropAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedLeftTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromLRedBackdropTape, LINE, HEAD_LINEAR);
                            route.addMovement(TURN, -0.9);
                            route.addEvent(Route.Action.WAIT, 0.3);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.1);
                            route.addMovement(TURN, 0.7);
                            route.addEvent(Route.Action.WAIT, 0.3);
                            route.addLocation(route.dropOnBackdropRedLeftBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            break;
                        case CENTER:
                            // Red Center Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addLocation(route.dropPixelRedCenterBackTapeAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedCenterTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromCRedBackdropTape, LINE, HEAD_LINEAR);
                            route.addMovement(TURN, -0.9);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropRedCenterBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);

                            break;
                        case RIGHT:
                            // Red Right Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedRightTapeBackdropAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedRightTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromRRedBackdropTape, LINE, HEAD_LINEAR);
                            route.addMovement(TURN, -0.9);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropRedRightBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            break;
                    }
                    break;
                case START_STACKS:
                    route.addLocation(route.start, START, HEAD_LINEAR);
                    switch (teamElement)
                    {
                        case LEFT:
                            // Red Left Stacks (7252)
                            route.addLocation(route.dropPixelRedLeftTapeStacks,LINE,HEAD_LINEAR);
                            route.addEvent(Route.Action.TANGENT, 300);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT,.2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            if(extraPixelGrab == GRAB_EXTRA_PIXEL)
                            {
                                route.addLocation(route.pickUpPixelStackLeftTapeRed, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::armDropSpikePos,1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                                route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
                            }
                            route.addLocation(route.backAwayFromRedTape, LINE, HEAD_LINEAR);
                            route.addLocation(route.moveTowardsRedBackdrop, LINE, HEAD_LINEAR,route.ninety);
                            route.addEvent(Route.Action.TANGENT, route.ninety);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.dropOnBackdropRedLeftStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            route.addFunction(route::armToDropHigher);
                            route.addLocation(route.dropOnBackdropRedLeftStacksHi, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            break;
                        case CENTER:
                            // Red Center Stacks (7252)
                            route.addLocation(route.dropPixelRedCenterTapeStacks,LINE,HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            if(extraPixelGrab == GRAB_EXTRA_PIXEL)
                            {
                                route.addLocation(route.pickUpPixelStackCenterTapeRed, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, .45);
                                route.addFunction(route::armDropSpikePos,1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .5);
                            }
                            route.addLocation(route.moveTowardsLoadStation, LINE, HEAD_LINEAR);
                            route.addLocation(route.moveTowardsRedBackdrop, LINE, HEAD_LINEAR,route.ninety);
                            route.addEvent(Route.Action.TANGENT, route.ninety);
                            route.addFunction(route::armToDrop, .5);
                            route.addLocation(route.dropOnBackdropRedCenterStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            route.addFunction(route::armToDropHigher);
                            route.addLocation(route.dropOnBackdropRedCenterStacksHi, LINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            break;
                        case RIGHT:
                            // Red Right Stacks (7252)
                            route.addLocation(route.dropPixelRedRightTapeStacksAdj,LINE,HEAD_LINEAR);
                            route.addLocation(route.dropPixelRedRightTapeStacks,LINE,HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, .2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            if(extraPixelGrab == GRAB_EXTRA_PIXEL)
                            {
                                route.addLocation(route.pickUpPixelStackRightTapeRed, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, .45);
                                route.addFunction(route::armDropSpikePos,1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                                route.addLocation(route.moveTowardsLoadStation, LINE, HEAD_LINEAR);
                            }
                            route.addLocation(route.moveTowardsRedBackdrop, LINE, HEAD_LINEAR,route.ninety);
                            route.addEvent(Route.Action.TANGENT, route.ninety);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.dropOnBackdropRedRightStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            route.addFunction(route::armToDropHigher);
                            route.addLocation(route.dropOnBackdropRedRightStacksHi, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            break;
                    }
            }
        }
        else
        {
            /* Blue Alliance */
            switch ((Field.StartPos)startPos)
            {
                case START_BACKDROP:
                    route.addLocation(route.start, SPLINE, HEAD_LINEAR);
                    switch (teamElement)
                    {
                        case LEFT:
                            // Blue Left Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallBlueBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueLeftTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromLBlueBackdropTapeAdj, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromLBlueBackdropTape, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.dropOnBackdropBlueLeftBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromBlueBackdropBk, LINE, HEAD_LINEAR);
                            break;
                        case CENTER:
                            // Blue Center Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallBlueBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueCenterBackTapeAdj, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueCenterTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromCBlueBackdropTape, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropOnBackdropBlueCenterBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromBlueBackdropBk, LINE, HEAD_LINEAR);
                            break;
                        case RIGHT:
                            // Blue Right Backdrop (7252)
                            route.addLocation(route.moveAwayFromWallBlueBackdrop, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueRightTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addLocation(route.moveAwayFromRBlueBackdropTape, LINE, HEAD_LINEAR);
                            route.addMovement(TURN, 2.5);
                            route.addEvent(Route.Action.WAIT, 0.3);
                            route.addLocation(route.dropOnBackdropBlueRightBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 0.5);
                            route.addLocation(route.reverseFromBlueBackdropBk, LINE, HEAD_LINEAR);
                            break;
                    }
                    break;
                case START_STACKS:
                    route.addLocation(route.start, START, HEAD_LINEAR);
                    switch (teamElement)
                    {
                        case LEFT:
                            // Blue Left Stacks (7252)
                            route.addLocation(route.dropPixelBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            route.addLocation(route.dropPixelBlueLeftTapeStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            if(extraPixelGrab == GRAB_EXTRA_PIXEL)
                            {
                                route.addLocation(route.pickUpPixelStackLeftTapeBlue, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                                route.addFunction(route::armDropSpikePos, .5);
                            }
                            route.addLocation(route.backAwayFromBlueTape, LINE, HEAD_LINEAR);
                            route.addLocation(route.moveTowardsBlueBackdrop, LINE, HEAD_LINEAR, route.ninety);
                            route.addEvent(Route.Action.TANGENT, route.ninety);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.dropOnBackdropBlueLeftStacks, SPLINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            route.addFunction(route::armToDropHigher);
                            route.addLocation(route.dropOnBackdropBlueLeftStacksHi, SPLINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            break;
                        case CENTER:
                            // Blue Center Stacks (7252)
                            route.addLocation(route.dropPixelBlueCenterTapeStacks,LINE,HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT,.2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            if (extraPixelGrab == NO_EXTRA_PIXEL)
                            {
                                route.addLocation(route.moveFromBlueCenterTapeStacks, LINE, HEAD_LINEAR);
                            }
                            else if(extraPixelGrab == GRAB_EXTRA_PIXEL)
                            {
                                route.addLocation(route.pickUpPixelStackCenterTapeBlue, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, .45);
                                route.addFunction(route::armDropSpikePos, 1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
//                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            }
                            route.addLocation(route.backAwayFromBlueTape, LINE, HEAD_LINEAR);
                            route.addLocation(route.moveTowardsBlueBackdrop, LINE, HEAD_LINEAR,route.ninety);
                            route.addEvent(Route.Action.TANGENT, route.ninety);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.dropOnBackdropBlueCenterStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            route.addFunction(route::armToDropHigher);
                            route.addLocation(route.dropOnBackdropBlueCenterStacksHi, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            break;
                        case RIGHT:
                            // Blue Right Stacks (7252)
                            route.addLocation(route.dropPixelBlueRightTapeStacks,LINE,HEAD_LINEAR);
                            route.addEvent(Route.Action.TANGENT, 300);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT,.2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            if (extraPixelGrab == NO_EXTRA_PIXEL)
                            {
                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            }
                            else if(extraPixelGrab == GRAB_EXTRA_PIXEL)
                            {
                                route.addLocation(route.pickUpPixelStackRightTapeBlue, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::allIntakesOn);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::armDropSpikePos,1);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            }
                            route.addLocation(route.backAwayFromBlueTape, LINE, HEAD_LINEAR);
                            route.addLocation(route.moveTowardsBlueBackdrop, LINE, HEAD_LINEAR,route.ninety);
                            route.addEvent(Route.Action.TANGENT, route.ninety);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.dropOnBackdropBlueRightStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            route.addFunction(route::armToDropHigher);
                            route.addLocation(route.dropOnBackdropBlueRightStacksHi, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT,.5);
                            break;
                    }
            }

        }

    }
}


