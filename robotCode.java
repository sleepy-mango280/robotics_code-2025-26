package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Sensor: Limelight3A FULL", group = "Sensor")
public class SensorLimelight3A extends LinearOpMode {

    private Limelight3A limelight;

    // ===== DRIVETRAIN =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== SHOOTER =====
    private DcMotor launcher;
    private DcMotor intake;

    // ===== FEEDERS (2x) =====
    private CRServo feederLeft;
    private CRServo feederRight;

    // ===== FEEDER STATE =====
    private boolean feederLast = false;
    private boolean feederActive = false;
    private long feederStartTime = 0;

    // ===== AUTO ALIGN TOGGLE =====
    private boolean autoAlignEnabled = false;
    private boolean autoAlignLast = false;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        // ===== DRIVETRAIN INIT =====
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== SHOOTER INIT =====
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        intake   = hardwareMap.get(DcMotor.class, "intake");

        // ===== FEEDERS INIT =====
        feederLeft  = hardwareMap.get(CRServo.class, "feederLeft");
        feederRight = hardwareMap.get(CRServo.class, "feederRight");

        feederLeft.setPower(0.0);
        feederRight.setPower(0.0);

        telemetry.addLine("Robot Ready");
        telemetry.addLine("Gamepad2: △ auto-align toggle | ○ intake | □ launcher | ✕ feed burst");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // LIMELIGHT STATUS 
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", status.getName());
            telemetry.addData("Temp", status.getTemp());
            telemetry.addData("CPU", status.getCpu());
            telemetry.addData("FPS", status.getFps());

            LLResult result = limelight.getLatestResult();
          
            //DRIVER INPUTS 
            double y  = -gamepad1.left_stick_y;   // forward/back
            double x  =  gamepad1.left_stick_x;   // strafe
            double rx =  gamepad1.right_stick_x;  // rotate


            //SHAPE-BUTTON CONTROLS
        
            // Launcher: (Square)
            launcher.setPower(gamepad2.x ? 1.0 : 0.0);

            // Intake: (Circle) forward only
            intake.setPower(gamepad2.b ? 1.0 : 0.0);

            // Feeders: (X) tap burst
            long FEEDER_RUN_MS = 1000;
            boolean feederButton = gamepad2.a;

            if (feederButton && !feederLast && !feederActive) {
                feederActive = true;
                feederStartTime = System.currentTimeMillis();
                feederLeft.setPower(1.0);
                feederRight.setPower(1.0);
            }
            if (feederActive &&
                    System.currentTimeMillis() - feederStartTime >= FEEDER_RUN_MS) {
                feederLeft.setPower(0.0);
                feederRight.setPower(0.0);
                feederActive = false;
            }
            feederLast = feederButton;

            // Auto-align toggle: △ (Triangle)
            boolean autoAlignButton = gamepad2.y;
            if (autoAlignButton && !autoAlignLast) {
                autoAlignEnabled = !autoAlignEnabled;
            }
            autoAlignLast = autoAlignButton;

            // ===============================
            // ===== ALIGNMENT & DISTANCE ====
            // ===============================
            boolean shoot = false;

            // Keep these as "input ..." so it errors until you edit them
            double TX_THRESHOLD = input ...;
            double TY_MIN       = input ...;
            double TY_MAX       = input ...;

            // Auto-align tuning (also left as input ...)
            double KP_TURN       = 0.05;
            double KP_DRIVE      = 0.07;
            double MAX_AUTO_TURN = 0.75;
            double MAX_AUTO_FWD  = 0.60;

            if (result != null && result.isValid()) {

                double tx = result.getTx();
                double ty = result.getTy();

                boolean aligned = Math.abs(tx) < TX_THRESHOLD;
                boolean inRange = (ty > TY_MIN && ty < TY_MAX);

                shoot = aligned && inRange;

                // ===== Driver assist message =====
                String assistMessage;
                if (tx > TX_THRESHOLD) {
                    assistMessage = "TURN RIGHT";
                } else if (tx < -TX_THRESHOLD) {
                    assistMessage = "TURN LEFT";
                } else if (ty < TY_MIN) {
                    assistMessage = "MOVE FORWARD";
                } else if (ty > TY_MAX) {
                    assistMessage = "MOVE BACK";
                } else {
                    assistMessage = "ON TARGET";
                }

                telemetry.addData("Driver Assist", assistMessage);
                telemetry.addData("Auto Align", autoAlignEnabled ? "ON (△ toggled)" : "OFF");
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("Aligned?", aligned);
                telemetry.addData("In Range?", inRange);

                // ===============================
                // ===== AUTO ALIGN OVERRIDE =====
                // ===============================
                if (autoAlignEnabled) {
                    // Turn command: rotate to reduce tx toward 0
                    double turnCmd = KP_TURN * tx;

                    // Range command: drive to bring ty into window
                    double tyError = 0.0;
                    if (ty < TY_MIN) {
                        tyError = (TY_MIN - ty);   // need forward
                    } else if (ty > TY_MAX) {
                        tyError = (TY_MAX - ty);   // negative -> need backward
                    }
                    double fwdCmd = KP_DRIVE * tyError;

                    // Clip
                    turnCmd = clip(turnCmd, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    fwdCmd  = clip(fwdCmd,  -MAX_AUTO_FWD,  MAX_AUTO_FWD);

                    // Override driver's forward/back and rotation (strafe stays manual)
                    y  = fwdCmd;
                    rx = turnCmd;

                    telemetry.addData("Auto y (fwd)", y);
                    telemetry.addData("Auto rx (turn)", rx);
                    telemetry.addLine("AUTO ALIGN ACTIVE");
                }

            } else {
                telemetry.addLine("NO TARGET");
                telemetry.addData("Auto Align", autoAlignEnabled ? "ON but no target" : "OFF");
            }

            // ===============================
            // ===== MECANUM DRIVE ===========
            // ===============================
            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr),
                    Math.max(Math.abs(bl), Math.abs(br)))));

            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            // ===============================
            // ===== SHOOT INDICATOR =========
            // ===============================
            if (shoot) {
                telemetry.addLine("====== SHOOT ======");
            } else {
                telemetry.addLine("---- DON'T SHOOT ----");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
