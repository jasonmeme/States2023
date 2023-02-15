package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class Distance extends OpMode {
    private DistanceSensor dSensor;
    @Override
    public void init() {
        dSensor = hardwareMap.get(DistanceSensor.class, "distance");
    }
    public void distance() {
        double value = dSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("distance: ", value);
        telemetry.update();
    }
    public void loop() {
        distance();
    }

    public void stop() {

    }

}


