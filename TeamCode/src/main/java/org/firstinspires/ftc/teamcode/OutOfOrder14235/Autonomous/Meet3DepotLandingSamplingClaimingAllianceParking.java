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
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled

@Autonomous

    public class Meet3DepotLandingSamplingClaimingAllianceParking extends OpMode {
         private ElapsedTime runtime = new ElapsedTime();

        private Gyroscope imu;
        private DcMotor leftWheel;
        private DcMotor rightWheel;
        private DcMotor centerWheel;
        private DcMotor linearActuator;
        private Servo markerDropper;
        private DcMotor linExt;
        //GoldAlignDetector
        private GoldAlignDetector detector;


    @Override
    public void init() {
        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!


    }

    @Override
    public void loop() {

        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftWheel = hardwareMap.get(DcMotor.class, "left_drive");
        rightWheel = hardwareMap.get(DcMotor.class, "right_drive");
        centerWheel = hardwareMap.get(DcMotor.class, "pulleyMotor");
        markerDropper = hardwareMap.get(Servo.class, "markerDropper");
        linearActuator = hardwareMap.get(DcMotor.class, "wormGear");
        linExt = hardwareMap.get(DcMotor.class, "linExt");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        runtime.reset();
/*
        markerDropper.setPosition(.3);
        linearActuator.setPower(-.7);
       Thread.sleep(4250);

        linearActuator.setPower(0);
        wait(1400);
        ShiftRight(.10);
        sleep(300);
        DriveForward(.3);
        sleep(625);
        StopDriving();
        sleep(100);
        TurnLeft(.2,-.2);
        sleep(580);
        StopDriving();
        sleep(1000);
        ShiftRight(1);
        sleep(2900);
        StopDriving();
        sleep(1000);
        markerDropper.setPosition(0.8);
        sleep(1000);
        markerDropper.setPosition(0.3);
        sleep(1000);
        markerDropper.setPosition(0.8);
        sleep(1000);
        markerDropper.setPosition(0.3);
        sleep(100);


        TurnRight(-.5,.5);
        sleep(1550);
        StopDriving();

        DriveForward(1);
        sleep(2900);
        StopDriving();
        linExt.setPower(-1);
        sleep(6700);


        TurnRight(-1.6,1.6);
        sleep(1400);
        DriveForward(.6);
        sleep(2000);
        linExt.setPower(-.5);
        sleep(7000);
        */
    }

    double DRIVE_POWER = 1.0;
    public void TurnLeft(double lPower, double rPower){
        leftWheel.setPower(lPower);
        rightWheel.setPower(rPower);

    }
    public void TurnRight(double lPower, double rPower){
        leftWheel.setPower(lPower);
        rightWheel.setPower(rPower);

    }

    public void DriveForward(double power){
        leftWheel.setPower(power);
        rightWheel.setPower(power);

    }
    public void DriveBackward(double power){
        leftWheel.setPower(-power);
        rightWheel.setPower(-power);

    }
    public void ShiftLeft(double power){
        centerWheel.setPower(power);
    }
    public void ShiftRight(double power){
        centerWheel.setPower(-power);
    }
    public void StopDriving(){
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        centerWheel.setPower(0);

    }

}
