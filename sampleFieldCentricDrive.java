@TeleOp(name = "FieldCentric TeleOp")
public class FieldCentricDriveTutorial extends OpMode {

    // Hardware variables
    DcMotor backLeft, backRight, frontLeft, frontRight;
    GoBildaPinpointDriver odo; // Requires the GoBilda repository [1, 2]

    @Override
    public void init() {
        // Hardware Mapping
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");

        // Optional: reverse motor directions based on your build
        // frontLeft.setDirection(DcMotor.Direction.REVERSE);

        // Pinpoint configuration [2, 3]
        odo.setOffsets(-84.0, -168.0); // Robot-specific measurements
        odo.setEncoderResolution(GoBildaPinpointDriver.EncoderResolution.FOUR_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Resetting position and IMU (heading) to ensure fresh data [3]
        odo.resetPosAndIMU();

        // Setting a 2D Pose for the starting position
        Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
        // odo.setStartingPosition(startingPos);

        telemetry.addData("Status", "Initialized");
    }

    public void moveRobot() {
        // 1. Inputs from Gamepad [4]
        // Vertical is negative on joysticks, so we negate it for 'forward'
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // 2. Grab heading from Pinpoint [4, 5]
        odo.getPosition();
        double heading = odo.getHeading(AngleUnit.RADIANS);

        // 3. Condensing Math for the Rotational Matrix [5]
        // This adjustment aligns the robot's heading with the driver's perspective
        double cos = Math.cos(Math.PI / 2 - heading);
        double sin = Math.sin(Math.PI / 2 - heading);

        // Applying the rotational matrix to translate global to local coordinates
        double globalForward = forward * cos - strafe * sin;
        double globalStrafe = forward * sin + strafe * cos;

        // 4. Calculating motor powers using Mecanum equations [6]
        double[] motorPowers = new double[7];
        motorPowers = globalForward + globalStrafe + rotate; // Front Left
        motorPowers[8] = globalForward - globalStrafe - rotate; // Front Right
        motorPowers[9] = globalForward - globalStrafe + rotate; // Back Left
        motorPowers[10] = globalForward + globalStrafe - rotate; // Back Right

        // 5. Setting Motor Powers [6]
        frontLeft.setPower(motorPowers);
        frontRight.setPower(motorPowers[8]);
        backLeft.setPower(motorPowers[9]);
        backRight.setPower(motorPowers[10]);
    }

    @Override
    public void loop() {
        // Refresh the odometry data constantly [11]
        odo.update();

        moveRobot();

        // Debugging Telemetry [11]
        telemetry.addData("Heading (Deg)", odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
