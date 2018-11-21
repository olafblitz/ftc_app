package org.firstinspires.ftc.teamcode.OutOfOrder14235.TeleOp;
/*Andrew Gao 11/21/18
  First TeleOp program just made, phasing out onbot java
  can use either twoWheelDrive or JoystickDrive*/
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "TeleOpV1")
public class TeleOpV1 extends OpMode{

    public DcMotor leftWheel;
    public DcMotor rightWheel;
    public DcMotor linearActuator;

    public void init(){
        leftWheel =  hardwareMap.get(DcMotor.class, "leftDrive");
        rightWheel =  hardwareMap.get(DcMotor.class, "rightDrive");
        linearActuator =  hardwareMap.get(DcMotor.class, "wormGear");

        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    public void loop (){


        TwoWheelDrive();
        MoveLinearActuator();
       // JoystickDrive();

    }
    public void JoystickDrive(){
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double trigger;
        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;
        trigger = gamepad1.right_trigger;

        left  = drive + turn;
        right = drive - turn;
        max = Math.max(Math.abs(left), Math.abs(right));

        if (max > 1.0)
        {
            left /= max;
            right /= max;

        }

        leftWheel.setPower(left*(1-trigger));
        rightWheel.setPower(right*(1-trigger));

    }

    public void TwoWheelDrive(){
        leftWheel.setPower(gamepad1.left_stick_y);
        rightWheel.setPower(gamepad1.right_stick_y);

    }

    public void MoveLinearActuator(){
        if (gamepad1.dpad_up){
            linearActuator.setPower(0.8);
        }
        else if (gamepad1.dpad_down){
            linearActuator.setPower(-0.7);
        }
    }

}
