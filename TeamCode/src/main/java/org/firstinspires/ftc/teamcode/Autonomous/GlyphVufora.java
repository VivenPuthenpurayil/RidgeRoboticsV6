/*package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.VuMarkTargetResult;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public class GlyphVufora extends LinearOpMode
{

    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

    params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
    params.VuforiaLicenseKey = "AeHK+j//////AAAAGV9JzI/9h0DLhZ7c7w4sN30lJRIpNPRyXVHdsqCX+XHpMysSwND71QWYT9YFkw" +
        "VxopMQaXnzmfWK7Sc2cSJJLPU9r2G/ioxim4UU4c4rPyvhtkOcZkaS6h" +
        "APo+aKdQVUsVkBsBbPIRcQOAEmp7oKqV0d/8pydpXHCAU" +
        "A18eNjdoEufCSugolPo84nHnEcEiklpqljewrCObyMTTwoftkpC" +
        "EabzJoHZ5s15Ztja9s9afEXBA5Vhp2OEcdxWQVoTHL5eFJog3faeMBSy" +
        "iU/NKjRNHv04w+P8lMnClXXLI8BiFVof8X5MDQPv8vFRHEADe8lCpnYDh1EGM0ZkFJv+gc59k4Ky1bIdUZYNvEto3Y5WRK";

    params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

    VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

    Vuforia.setHint(HINT.HINT_MAX_SIMALTANEOUS_IMAGE_TARGETS,1); //how many targets viewed at a time

    VuforiaTrackables glyphs = vuforia.loadTrackablesFromAsset(“GlyphTracking_OT”);

    glyphs.get(0).setName(“glyphBrown”);

    VuforiaTrackableDefaultListener glyphBrown = (VuforiaTrackableDefaultListener) glyphs.get(0).getListener();
    waitForStart();
glyphs.activate();  // would be at beginning of code

// next lines would be edited to be intergrated into program


    VectorF angles = anglesFromTarget(glyphBrown);
    VectorF trans = navOffWall(glyphBrown.getPose().getTranslation(),Math.toDegrees(angles.get()) =90, new VectorF(500,0,0));

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image) {
        float[] data = image.getRawPose().getData();
        float[][] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float) thetaX, (float) thetaY, (float) thetaZ);

        while (opModeIsActive && glyphBrown.getPose() != null) {
            VectorF trans = navOffWall(glyphBrown.getPose().getTranslation(),Math.toDegrees(angles.get()) =90, new VectorF(500,0,0));
            telemetry.addData("Left/Right: ", trans.get(0));
            telemetry.addData("Up/Down: ", trans.get(1));
            telemetry.addData("Front/Back: ", trans.get(2));
             sleep(2000);
             telemetry.update();

        }

    }
*/
