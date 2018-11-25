package org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous;

/*
Copyright 2018 FIRST Tech Challenge Team 14235

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous

public class AutonomousWithLinearActuator extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor wormDrive;

    @Override
    public void runOpMode() throws InterruptedException{
        imu = hardwareMap.get(Gyroscope.class, "imu");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        wormDrive =  hardwareMap.get(DcMotor.class, "wormGear");

        left_drive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        wormDrive.setPower(-.7);
        sleep(4000);
        DriveBackward(.7);
        sleep(500);


    }

    double DRIVE_POWER = 1.0;
    public void TurnLeft(double lPower, double rPower){
        left_drive.setPower(lPower);
        right_drive.setPower(rPower);

    }
    public void TurnRight(double lPower, double rPower){
        left_drive.setPower(lPower);
        right_drive.setPower(rPower);

    }
    public void DriveForward(double power){
        left_drive.setPower(power);
        right_drive.setPower(power);

    }
    public void DriveBackward(double power){
        left_drive.setPower(-power);
        right_drive.setPower(-power);

    }
    public void StopDriving(){
        DriveForward(0);
    }

}
