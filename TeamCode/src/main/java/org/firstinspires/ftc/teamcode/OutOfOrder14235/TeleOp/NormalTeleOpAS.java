package org.firstinspires.ftc.teamcode.OutOfOrder14235.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous.HardwareRobot;

import java.util.Locale;

@TeleOp

public class NormalTeleOpAS extends LinearOpMode {
    HardwareRobot robot;

    private Gyroscope imu;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DcMotor centerWheel;
    private DcMotor linearActuator;
    private Servo markerDropper;
    private DcMotor linExt;
    DistanceSensor distanceBottom;
    ColorSensor colorSideLeft;
    DistanceSensor distanceSideLeft;
    ColorSensor colorSideRight;
    DistanceSensor distanceSideRight;
    DigitalChannel touchLeft;
    DigitalChannel  touchRight;
    ColorSensor colorMarker;
    DistanceSensor distanceMarker;

    private DcMotor flipper;

    @Override
    public void runOpMode() {
        robot = new HardwareRobot();
        robot.init(hardwareMap);
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double trigger;

        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftWheel = hardwareMap.get(DcMotor.class, "left_drive");
        rightWheel = hardwareMap.get(DcMotor.class, "right_drive");
        centerWheel = hardwareMap.get(DcMotor.class, "pulleyMotor");
        markerDropper = hardwareMap.get(Servo.class, "markerDropper");
        linearActuator = hardwareMap.get(DcMotor.class, "wormGear");
        linExt = hardwareMap.get(DcMotor.class, "linExt");
        flipper = hardwareMap.get(DcMotor.class, "flipper");
        distanceBottom = hardwareMap.get(DistanceSensor.class, "distanceBottom");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distanceBottom;
        colorSideLeft = hardwareMap.get(ColorSensor.class, "colorDistanceLeft");
        colorSideRight = hardwareMap.get(ColorSensor.class, "colorDistanceRight");

        distanceSideLeft = hardwareMap.get(DistanceSensor.class, "colorDistanceLeft");
        distanceSideRight = hardwareMap.get(DistanceSensor.class, "colorDistanceRight");
        touchLeft = hardwareMap.get(DigitalChannel.class, "touchSensorLeft");
        touchLeft.setMode(DigitalChannel.Mode.INPUT);
        touchRight = hardwareMap.get(DigitalChannel.class, "touchSensorRight");
        touchRight.setMode(DigitalChannel.Mode.INPUT);
        colorMarker = hardwareMap.get(ColorSensor.class, "colorMarker");
        distanceMarker = hardwareMap.get(DistanceSensor.class, "colorMarker");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        float hsvValuesLeft[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float valuesLeft[] = hsvValuesLeft;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTORLEFT = 255;
        float hsvValuesRight[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float valuesRight[] = hsvValuesRight;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTORRIGHT = 255;
        telemetry.addData("Status", "Ready to start!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Color.RGBToHSV((int) (colorSideLeft.red() * SCALE_FACTORLEFT),
                    (int) (colorSideLeft.green() * SCALE_FACTORLEFT),
                    (int) (colorSideLeft.blue() * SCALE_FACTORLEFT),
                    hsvValuesLeft);

            // send the info back to driver station using telemetry function.

            telemetry.addData("LEFT Red  ", colorSideLeft.red());
            telemetry.addData("LEFT Blue ", colorSideLeft.blue());

            Color.RGBToHSV((int) (colorSideRight.red() * SCALE_FACTORRIGHT),
                    (int) (colorSideRight.green() * SCALE_FACTORRIGHT),
                    (int) (colorSideRight.blue() * SCALE_FACTORRIGHT),
                    hsvValuesRight);

            // send the info back to driver station using telemetry function.

            telemetry.addData("RIGHT Red  ", colorSideRight.red());
            telemetry.addData("RIGHT Blue ", colorSideRight.blue());


            telemetry.addData("range", String.format("%.01f centimeters bottom distance sensor", distanceBottom.getDistance(DistanceUnit.CM)));
            telemetry.addData("linear actuator encoder position: ", linearActuator.getCurrentPosition());
            telemetry.update();
            // trigger = gamepad1.right_trigger;
            //drive = -gamepad1.left_stick_y;
            //  turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            //left  = drive + turn;
            //right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            //  max = Math.max(Math.abs(left), Math.abs(right));
            //if (max > 1.0)
            //{
            //    left /= max;
            //    right /= max;
            // }

            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            left  = drive + turn;
            right = drive - turn;
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            leftWheel.setPower(left);
            rightWheel.setPower(right);
            if(gamepad2.right_trigger>0){
                flipper.setPower(.4);
            }
            else if(gamepad2.left_trigger>0){
                flipper.setPower(-.4);

            }
            else{
                flipper.setPower(0);
            }

            if(gamepad1.y){
                linearActuator.setPower(-1);
            }
            else if(gamepad1.a){
                linearActuator.setPower(1);
            }
            else{
                linearActuator.setPower(0);
            }
            if(gamepad1.x){
                centerWheel.setPower(.7);
            }
            else if(gamepad1.b){
                centerWheel.setPower(-.7);
            }
            else{
                centerWheel.setPower(0);
            }
            if (gamepad1.dpad_left){
                markerDropper.setPosition(.3);
            }
            else if (gamepad1.dpad_right){
                markerDropper.setPosition(0.75);

            }
            if(gamepad2.y){
                linExt.setPower(-1);
            }
            else if(gamepad2.a){
                linExt.setPower(1);
            }
            else{
                linExt.setPower(0);
            }
            if(gamepad1.dpad_down){
                robot.sideArm.setPower(-1);
            }
            else if(gamepad1.dpad_up){
                robot.sideArm.setPower(1);
            }
            else{
                robot.sideArm.setPower(0);
            }


        }
        telemetry.addData("Status", "Running");

        telemetry.update();

    }


}

