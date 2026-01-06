package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sensor: Limelight3A Driver Assist", group = "Sensor")
public class SensorLimelight3ADriverAssist extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            boolean shoot = false;

            //  INPUT VALUES HERE 
            double TX_THRESHOLD = input ...;   // degrees left/right tolerance (e.g., 1.0)
            double TY_MIN = input ...;         // min vertical angle for “close enough”
            double TY_MAX = input ...;         // max vertical angle for “close enough”

            if (result != null && result.isValid()) {

                double tx = result.getTx();  // horizontal offset
                double ty = result.getTy();  // vertical offset (proxy for distance)

                // ===== DRIVER DIRECTION ASSIST (MESSAGE ONLY) =====
                String assistMessage;

                // Priority: left/right first, then distance
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

                //  SHOOT DECISION 
                boolean aligned = Math.abs(tx) < TX_THRESHOLD;
                boolean inRange = (ty > TY_MIN && ty < TY_MAX);
                shoot = aligned && inRange;

                // Optional debug
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("Aligned?", aligned);
                telemetry.addData("In Range?", inRange);

            } else {
                telemetry.addData("Driver Assist", "NO TARGET");
            }

            //  SHOOT / DON'T SHOOT 
            if (shoot) {
                telemetry.addLine("====== SHOOT ======");
            } else {
                telemetry.addLine("---- DON'T SHOOT ----");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
