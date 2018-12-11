package org.firstinspires.ftc.teamcode.OutOfOrder14235.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

public class TwoStickTeleOp extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DcMotor sideArm;
    private DcMotor linearActuator;
    private Servo markerDropper;



    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double trigger;
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftWheel = hardwareMap.get(DcMotor.class, "left_drive");
        rightWheel = hardwareMap.get(DcMotor.class, "right_drive");
        sideArm = hardwareMap.get(DcMotor.class, "pulleyMotor");
        markerDropper = hardwareMap.get(Servo.class, "markerDropper");
        linearActuator = hardwareMap.get(DcMotor.class, "wormGear");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready to start!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // trigger = gamepad1.right_trigger;
            right = -gamepad1.right_stick_y;
            left = -gamepad1.left_stick_y;

            leftWheel.setPower(left);
            rightWheel.setPower(right);

            if(gamepad1.a){
                linearActuator.setPower(-1);
            }
            else if(gamepad1.b){
                linearActuator.setPower(1);
            }
            else{
                linearActuator.setPower(0);
            }
            if(gamepad1.x){
                sideArm.setPower(.7);
            }
            else if(gamepad1.y){
                sideArm.setPower(-.7);
            }
            else{
                sideArm.setPower(0);
            }
            if (gamepad1.dpad_left){
                markerDropper.setPosition(.25);
            }
            else if (gamepad1.dpad_right){
                markerDropper.setPosition(1);

            }

            telemetry.addData("Status", "Running");

            telemetry.update();

        }


    }

}
