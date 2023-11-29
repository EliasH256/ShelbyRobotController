package org.firstinspires.ftc.teamcode.field;



import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;
import static org.firstinspires.ftc.teamcode.field.Field.ParkLocation.*;

public class MoveToPark {
    Route route;
    public MoveToPark(Route constructorRoute) {
        route = constructorRoute;
   }

    public void makeTraj(Field.ParkLocation parkPos, Field.Alliance alliance)
    {
        if (alliance == Field.Alliance.BLUE)
        {
            if(parkPos == WALLSIDE_PARK)
            {
                route.addEvent(Route.Action.WAIT, 0.3);
                route.addLocation(route.parkWallAdj,LINE,HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::armDropSpikePos);
                route.addEvent(Route.Action.WAIT, 0.2);
                route.addLocation(route.parkWall,LINE,HEAD_LINEAR);
            }
            else if(parkPos == DOORSIDE_PARK)
            {
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
            if(parkPos == WALLSIDE_PARK)
            {
                route.addLocation(route.parkWallAdjRed,LINE,HEAD_LINEAR);
                route.addEvent(Route.Action.WAIT, 0.1);
                route.addFunction(route::armDropSpikePos);
                route.addEvent(Route.Action.WAIT, 0.2);
                route.addLocation(route.parkWallRed,LINE,HEAD_LINEAR);
            }
            else if(parkPos == DOORSIDE_PARK)
            {
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


