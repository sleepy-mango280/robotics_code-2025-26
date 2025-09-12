package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "mecanumDrive", group = "MyTeam")
public class mechanumDrive extends LinearOpMode {

    
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    
    @Override
    public void runOpMode() {

       
        telemetry.addData("Status", "Initializing Robot...");
        telemetry.update();

      
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft  = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

       
        waitForStart();

    
        while (opModeIsActive()) {

         
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

           
            double speedMultiplier = 1.0;
            if (gamepad1.right_trigger > 0) {
                speedMultiplier = 1.0 - (gamepad1.right_trigger * 0.75);
                // Adjusting the multiplier from 1.0 down to 0.25 (1.0 - 0.75) for slower movement.
            } else if (gamepad1.left_trigger > 0) {
                speedMultiplier = 1.0; // Left trigger for full speed
            }

            // These are the special calculations for Mecanum wheels. They combine the
            // forward, sideways, and rotational movements into a single power value
            // for each motor.
            double frontLeftPower = (y + x + rx) * speedMultiplier;
            double backLeftPower = (y - x + rx) * speedMultiplier;
            double frontRightPower = (y - x - rx) * speedMultiplier;
            double backRightPower = (y + x - rx) * speedMultiplier;

            
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                backLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backRightPower /= maxPower;
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.addData("Speed Multiplier", speedMultiplier);
            telemetry.update();
        }
    }
}
