package org.firstinspires.ftc.teamcode.util;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;

@SuppressWarnings("unused")
public class PreferenceMgr
{
   private CommonUtil com;
   private SharedPreferences sharedPreferences;

   private String teamNumber     = "7253";
   private String allianceColor  = "RED";
   private String startPosition  = "START_A";
   private String parkPosition   = "PARK_A";
   private String robotConfig    = Drivetrain.DrivetrainType.RWD_2_2X40.toString();
   private int delay             = 0;
   private int debug             = 1;

   public String getTeamNumber()
   {
      return teamNumber;
   }

   public String getAllianceColor()
   {
      return allianceColor;
   }

   public String getStartPosition()
   {
      return startPosition;
   }

   public String getParkPosition()
   {
      return parkPosition;
   }

   public String getRobotConfig()
   {
      return robotConfig;
   }

   public int getDelay()
   {
      return delay;
   }

   public int getDebug()
   {
      return debug;
   }

   public PreferenceMgr()
   {
      com = CommonUtil.getInstance();
   }

   public void readPrefs()
   {
      sharedPreferences = PreferenceManager.getDefaultSharedPreferences(com.getContext());

      teamNumber    = sharedPreferences.getString("shelby.7253.Autonomous.TeamNumber",
              teamNumber);
      allianceColor = sharedPreferences.getString("shelby.7253.Autonomous.Alliance",
              allianceColor);
      startPosition = sharedPreferences.getString("shelby.7253.Autonomous.StartPosition",
              startPosition);
      parkPosition  = sharedPreferences.getString("shelby.7253.Autonomous.ParkPosition",
              parkPosition);
      robotConfig   = sharedPreferences.getString("shelby.7253.Autonomous.RobotConfig",
              robotConfig);
      delay         = Integer.parseInt(sharedPreferences.getString("shelby.7253.Autonomous.Delay",
              Integer.toString(delay)));
      debug         = Integer.parseInt(sharedPreferences.getString("shelby.7253.Autonomous.Debug",
              Integer.toString(debug)));

      writePrefs();
   }

   private void writePrefs()
   {
      //write the options to sharedpreferences
      SharedPreferences.Editor editor = sharedPreferences.edit();
      editor.putString("shelby.7253.Autonomous.TeamNumber",    teamNumber);
      editor.putString("shelby.7253.Autonomous.Alliance",      allianceColor);
      editor.putString("shelby.7253.Autonomous.StartPosition", startPosition);
      editor.putString("shelby.7253.Autonomous.ParkPosition",  parkPosition);
      editor.putString("shelby.7253.Autonomous.RobotConfig",   robotConfig);
      editor.putString("shelby.7253.Autonomous.Delay",         String.valueOf(delay));
      editor.putString("shelby.7253.Autonomous.Debug",         String.valueOf(debug));
      editor.apply();
   }

}
