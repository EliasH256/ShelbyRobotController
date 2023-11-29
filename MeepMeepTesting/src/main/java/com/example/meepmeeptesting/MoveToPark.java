package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Route.Heading.HEAD_LINEAR;
import static com.example.meepmeeptesting.Route.Heading.HEAD_SPLINE;
import static com.example.meepmeeptesting.Route.Movement.LINE;
import static com.example.meepmeeptesting.Route.Movement.TURN;
import static com.example.meepmeeptesting.Route.Movement.SPLINE;
import static com.example.meepmeeptesting.Field.Highways.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Locale;

public class MoveToPark {
    Route route;
    public MoveToPark(Route constructorRoute) {
        route = constructorRoute;
   }

    public void makeTraj(Field.Highways parkPos, Field.Alliance alliance) {
        if (alliance == Field.Alliance.BLUE)
        {
            if(parkPos == WALL){
                route.addMovement(TURN, 0.5);
                route.addEvent(Route.Action.WAIT, 0.3);
                route.addLocation(route.parkWallAdj,LINE,HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::armDropSpikePos);
                route.addEvent(Route.Action.WAIT, 0.2);
                route.addLocation(route.parkWall,LINE,HEAD_LINEAR);
            }
            else if(parkPos == DOOR){
                route.addMovement(TURN, -0.5);
                route.addEvent(Route.Action.WAIT, 0.3);
                route.addLocation(route.parkDoorAdj,LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::armDropSpikePos);
                route.addEvent(Route.Action.WAIT, 0.2);
                route.addLocation(route.parkDoor,LINE, HEAD_LINEAR);
            }

        }
        else if (alliance == Field.Alliance.RED)
        {
            if (route.startPos == Field.StartPos.START_STACKS && route.teamElement == Route.TeamElement.RIGHT)
            {
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::armDropSpikePos);
                route.addEvent(Route.Action.WAIT, 0.2);
                route.addLocation(route.parkWallRedRight,LINE,HEAD_LINEAR);
            }
            else if(parkPos == WALL){
                route.addMovement(TURN, -0.5);
                route.addEvent(Route.Action.WAIT, 0.3);
                route.addLocation(route.parkWallAdjRed,LINE,HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::armDropSpikePos);
                route.addEvent(Route.Action.WAIT, 0.2);
                route.addLocation(route.parkWallRed,LINE,HEAD_LINEAR);
            }
            else if(parkPos == DOOR){
                route.addMovement(TURN, 0.5);
                route.addEvent(Route.Action.WAIT, 0.3);
                route.addLocation(route.parkDoorAdjRed,LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::armDropSpikePos);
                route.addEvent(Route.Action.WAIT, 0.2);
                route.addLocation(route.parkDoorRed,LINE, HEAD_LINEAR);
            }

        }

    }
}


