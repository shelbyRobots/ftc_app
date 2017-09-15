package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.ShelbyBot;

/**
 * Created by Martin on 11/9/2016.
 */
@Autonomous(name="Gyro Test", group="Auton")
public class GyroTest extends LinearOpMode
{
    public void runOpMode() throws InterruptedException
    {
        robot.init(this);

        waitForStart();

     //   move(.3, -.3);

        while(opModeIsActive())
        {
            telemetry.addData("Gyro Heading : ", robot.gyro.getHeading());
            telemetry.update();

            Thread.sleep(10);
            idle();
        }

    }
    private ShelbyBot robot = new ShelbyBot();

    void move(double ldp, double rdp)
    {
        robot.leftMotor.setPower(ldp);
        robot.rightMotor.setPower(rdp);
    }

}
