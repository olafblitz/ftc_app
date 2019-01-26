package org.firstinspires.ftc.teamcode.OutOfOrder14235.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class NormalTeleOpAS extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DcMotor centerWheel;
    private DcMotor linearActuator;
    private Servo markerDropper;
    private DcMotor linExt;
    private CRServo intakeSpinner;
    private CRServo intakeFlipper;

    @Override
    public void runOpMode() {

        double left;
        double right;
        double drive;
        double turn;
        double max;
        double trigger;
        intakeSpinner = hardwareMap.get(CRServo.class, "intake");
        intakeFlipper = hardwareMap.get(CRServo.class, "intakeFlipper");

        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftWheel = hardwareMap.get(DcMotor.class, "left_drive");
        rightWheel = hardwareMap.get(DcMotor.class, "right_drive");
        centerWheel = hardwareMap.get(DcMotor.class, "pulleyMotor");
        markerDropper = hardwareMap.get(Servo.class, "markerDropper");
        linearActuator = hardwareMap.get(DcMotor.class, "wormGear");
        linExt = hardwareMap.get(DcMotor.class, "linExt");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready to start!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("encoder position: ", linearActuator.getCurrentPosition());
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
                intakeFlipper.setPower(.4);
            }
            else if(gamepad2.left_trigger>0){
                intakeFlipper.setPower(-.4);

            }
            else{
                intakeFlipper.setPower(0);
            }
            if(gamepad2.left_bumper){
                intakeSpinner.setPower(1);
            }
            else if (gamepad2.right_bumper ){
                intakeSpinner.setPower(-1);
            }
            else{
                intakeSpinner.setPower(0);
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
        }
        telemetry.addData("Status", "Running");

        telemetry.update();

    }


}

