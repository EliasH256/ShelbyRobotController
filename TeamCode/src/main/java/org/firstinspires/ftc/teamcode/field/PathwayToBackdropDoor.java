package org.firstinspires.ftc.teamcode.field;


import static org.firstinspires.ftc.teamcode.field.Route.Heading.HEAD_LINEAR;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.LINE;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.TURN;

public class PathwayToBackdropDoor
{
    Route route;
    public PathwayToBackdropDoor(Route constructorRoute)
    {
        route = constructorRoute;
    }

    public void makeTraj(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, Field.stacksSideExtraPixelGrab extraPixelGrab)
    {
        if (alliance == Field.Alliance.BLUE)
        {
            route.addEvent(Route.Action.WAIT, 60);
        }
        else if (alliance == Field.Alliance.RED)
        {
            route.addEvent(Route.Action.WAIT, 60);
        }

    }
}


