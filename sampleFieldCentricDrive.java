package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp

public class fieldCentricDrive extends OpMode {

    GoBildaPinpointDriver odo;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        SparkFunOTOS.Pose2D startingPosition = new SparkFunOTOS.Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X Position (mm)", odo.getXOffset());
        telemetry.addData("Y Position (mm)", odo.getYOffset());
        telemetry.addData("Device Version", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        public void moveRobot() {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            SparkFunOTOS.Pose2D pos = odo.getPosition();
            double heading = pos.getHeading(AngleUnit.RADIANS);

            double cosAngle = Math.cos(Math.PI/2)-heading;
            double sinAngle = Math.sin(Math.PI/2)-heading;

            double globalStrafe = -forward * sinAngle + strafe * cosAngle;
            double globalForward = forward * cosAngle + strafe * sinAngle;

            double [] newWheelSpeeds = new double[4];
            newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
            newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
            newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
            newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

            frontLeft.setPower(newWheelSpeeds[0]);
            frontRight.setPower(newWheelSpeeds[1]);
            backLeft.setPower(newWheelSpeeds[2]);
            backRight.setPower(newWheelSpeeds[3]);
            telemetry.addData("XPos", pos.getX(DistanceUnit.MM));
            telemetry.addData("YPos", pos.getY(DistanceUnit.MM));
            telemetry.addData("Heading", heading);
            telemetry.addData("Forward speed", globalForward);
            telemetry.addData("Strafe speed", globalStrafe);
            telemetry.update();

        }

        public void loop() {
            moveRobot();

            Pose2D pos = odo.getPosition();
            odo.update();
        }
    }
}
