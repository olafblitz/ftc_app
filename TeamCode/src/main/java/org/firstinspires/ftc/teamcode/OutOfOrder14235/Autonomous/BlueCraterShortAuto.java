package org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous;
/*Basic TeleOp Program by Andrew Gao on 11/21/18
Goes down from lander, and goes into pit
not finished or tested
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class BlueCraterShortAuto extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor linearActuator;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() throws InterruptedException{

        imu = hardwareMap.get(Gyroscope.class, "imu");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        linearActuator =  hardwareMap.get(DcMotor.class, "wormGear");

        right_drive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Path0",  "Starting at %7d :%7d", left_drive.getCurrentPosition(), right_drive.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        DownLinearActuator();
        sleep(3000);
        encoderDrive(DRIVE_SPEED, -6, -6, 4.0);  // S3: Reverse 6 Inches with 4 Sec timeout
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
        linearActuator.setPower(-.7);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            left_drive.setTargetPosition(newLeftTarget);
            right_drive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left_drive.setPower(Math.abs(speed));
            right_drive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left_drive.isBusy() && right_drive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        left_drive.getCurrentPosition(),
                        right_drive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            left_drive.setPower(0);
            right_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
