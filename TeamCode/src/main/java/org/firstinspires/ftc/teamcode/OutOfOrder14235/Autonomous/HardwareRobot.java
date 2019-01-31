/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareRobot
{
    /* Public OpMode members. */
    BNO055IMU imu;
    public DcMotor leftWheel;
    public DcMotor rightWheel;
    public DcMotor centerWheel;
    public DcMotor linearActuator;
    public Servo markerDropper;
    ColorSensor colorSideLeft;
    DistanceSensor distanceSideLeft;
    ColorSensor colorSideRight;
    DistanceSensor distanceSideRight;
    DigitalChannel  touchLeft;
    DigitalChannel  touchRight;
    DistanceSensor sensorSideRange;
    ColorSensor colorMarker;
    DistanceSensor distanceMarker;

public DcMotor flipper;
    public DcMotor linExt;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        imu = hwMap.get(BNO055IMU.class, "imuBase");

        leftWheel = hwMap.get(DcMotor.class, "left_drive");
        rightWheel = hwMap.get(DcMotor.class, "right_drive");
        centerWheel = hwMap.get(DcMotor.class, "pulleyMotor");
        markerDropper = hwMap.get(Servo.class, "markerDropper");

        linearActuator = hwMap.get(DcMotor.class, "wormGear");
        linExt = hwMap.get(DcMotor.class, "linExt");
        colorSideLeft = hwMap.get(ColorSensor.class, "colorDistanceLeft");
        colorSideRight = hwMap.get(ColorSensor.class, "colorDistanceRight");

        distanceSideLeft = hwMap.get(DistanceSensor.class, "colorDistanceLeft");
        distanceSideRight = hwMap.get(DistanceSensor.class, "colorDistanceRight");
        touchLeft = hwMap.get(DigitalChannel.class, "touchSensorLeft");
        touchLeft.setMode(DigitalChannel.Mode.INPUT);
        touchRight = hwMap.get(DigitalChannel.class, "touchSensorRight");
        touchRight.setMode(DigitalChannel.Mode.INPUT);
        colorMarker = hwMap.get(ColorSensor.class, "colorMarker");
        distanceMarker = hwMap.get(DistanceSensor.class, "colorMarker");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorSideRange;


        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void deLatch() {
        linearActuator.setTargetPosition(16000);
    }



    public void stopDrive(){
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
    }

    public float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    }


}




