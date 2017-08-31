package org.firstinspires.ftc.teamcode;

public class RouteStates
{
    enum SEG_STATES
    {
        START,
        PRSHT,
        SHOOT,
        AIMTO,
        PRESS,
        PARK
    }

    enum SUB_STATES
    {
        TURN,
        MOVE,
        ACT
    }

    enum PUSH_STATES
    {
        SCAN,
        BECN,
        PUSH,
        RVRS
    }
}
