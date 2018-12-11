
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class BlueAutonomousLongWithMarkerAndEnemyCrater extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor linearActuator;
    private DcMotor sideArm;
    private Servo markerDropper;
    @Override
    public void runOpMode() throws InterruptedException{
        imu = hardwareMap.get(Gyroscope.class, "imu");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        linearActuator =  hardwareMap.get(DcMotor.class, "wormGear");
        sideArm = hardwareMap.get(DcMotor.class, "pulleyMotor");
        markerDropper = hardwareMap.get(Servo.class,"markerDropper" );
        left_drive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        markerDropper.setPosition(.25);

        linearActuator.setPower(-.22);
        sleep(14900);
        linearActuator.setPower(0);
        sleep(1300);
        DriveBackward(.8);
        sleep(2300);
        DriveBackward(0);
        sleep(100);
        markerDropper.setPosition(1.7);
        sleep(300);
        markerDropper.setPosition(.35);
        sleep(300);
        DriveForward(.5);
        sleep(600);
        TurnRight(.5,-.5);
        sleep(1600);
        DriveForward(.8);
        sleep(3600);


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
