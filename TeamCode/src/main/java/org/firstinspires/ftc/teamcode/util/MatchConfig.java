package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.field.Field;

public class MatchConfig
{
    public Field.Alliance getAlliance()
    {
        return alliance;
    }

    public void setAlliance(Field.Alliance alliance)
    {
        this.alliance = alliance;
    }

    public Field.StartPos getStartPos()
    {
        return startPos;
    }

    public void setStartPos(Field.StartPos startPos)
    {
        this.startPos = startPos;
    }

    public Field.ParkChoice getParkPos()
    {
        return parkPos;
    }

    public void setParkPos(Field.ParkChoice parkPos)
    {
        this.parkPos = parkPos;
    }

    public String getBotName()
    {
        return botName;
    }

    public void setBotName(String botName)
    {
        this.botName = botName;
    }

    public double getDelay()
    {
        return delay;
    }

    public void setDelay(double delay)
    {
        this.delay = delay;
    }

    private Field.Alliance   alliance;
    private Field.StartPos   startPos;
    private Field.ParkChoice parkPos;
    private String           botName = "";
    private double           delay = 0.0;
}
