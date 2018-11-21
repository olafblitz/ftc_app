package org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous;
/*Basic TeleOp Program by Andrew Gao on 11/21/18
Goes down from lander, and goes into pit
not finished or tested
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class BlueCraterShortAuto extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor linearActuator;

    @Override
    public void runOpMode() throws InterruptedException{
        imu = hardwareMap.get(Gyroscope.class, "imu");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        linearActuator =  hardwareMap.get(DcMotor.class, "wormGear");

        right_drive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        DownLinearActuator();
        sleep(1000);
        TurnRight(-150,100);
        sleep(1000);
        DriveBackward(1);
        sleep(1500);
        StopDriving();
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
        left_drive.setPower(0);
        right_drive.setPower(0);
    }

    public void DownLinearActuator(){
        linearActuator.setPower(-.6);
    }
}
