package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Field.FirstLocation.*;
import static com.example.meepmeeptesting.Field.StartPos.*;
import static com.example.meepmeeptesting.Route.Heading.HEAD_DEFAULT;
import static com.example.meepmeeptesting.Route.Heading.HEAD_LINEAR;
import static com.example.meepmeeptesting.Route.Heading.HEAD_SPLINE;
import static com.example.meepmeeptesting.Route.Movement.LINE;
import static com.example.meepmeeptesting.Route.Movement.SPLINE;
import static com.example.meepmeeptesting.Route.Movement.START;
import static com.example.meepmeeptesting.Route.Movement.TURN;
import static com.example.meepmeeptesting.Route.TeamElement.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Locale;

public class DropOnTapeThenBackdrop {
    Route route;
    public DropOnTapeThenBackdrop(Route constructorRoute) {
route = constructorRoute;
    }

    public void makeTraj(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, Field.FirstLocation firstLocation) {
        qualifierRoute(teamElement, alliance,startPos,firstLocation);
    }

    private void qualifierRoute(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, Field.FirstLocation firstLocation)
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
                            if (firstLocation == BACKDROP) 
							{
                            route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
                            }
						    else if(firstLocation == PIXEL_WALL)
							{
                                route.addLocation(route.pickUpPixelStackLeftTapeRed, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
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
                            if (firstLocation == BACKDROP) 
							{
                                route.addLocation(route.moveFromRedCenterTapeStacks, LINE, HEAD_LINEAR);
                            }
							else if(firstLocation == PIXEL_WALL)
							{
                                route.addLocation(route.pickUpPixelStackCenterTapeRed, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
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
                            if (firstLocation == BACKDROP)
                            {
                                route.addLocation(route.moveFromRedRightTapeStacks, LINE, HEAD_LINEAR);
                            }
                            else if(firstLocation == PIXEL_WALL)
                            {
                                route.addLocation(route.pickUpPixelStackRightTapeRed, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
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
                            if (firstLocation == BACKDROP)
							{
                                route.addLocation(route.moveFromBlueLeftTapeStacks, LINE, HEAD_LINEAR);
                            }
							else if(firstLocation == PIXEL_WALL)
							{
                                route.addLocation(route.pickUpPixelStackLeftTapeBlue, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
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
                            if (firstLocation == BACKDROP)
                            {
                                route.addLocation(route.moveFromBlueCenterTapeStacks, LINE, HEAD_LINEAR);
                            }
                            else if(firstLocation == PIXEL_WALL)
                            {
                                route.addLocation(route.pickUpPixelStackCenterTapeBlue, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
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
                            if (firstLocation == BACKDROP)
							{
                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            }
							else if(firstLocation == PIXEL_WALL)
							{
                                route.addLocation(route.pickUpPixelStackRightTapeBlue, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
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
}


