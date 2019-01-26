package org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
public class Basic extends LinearOpMode {
    HardwareRobot robot;
    public void runOpMode() {
        robot = new HardwareRobot();
        robot.init(hardwareMap);
        waitForStart();

    }

    public void waiting(long millis){
        long t = System.currentTimeMillis();
        while(opModeIsActive()&&System.currentTimeMillis()-t<millis){

        }
    }

}
