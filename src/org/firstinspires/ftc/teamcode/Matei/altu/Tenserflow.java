//package org.firstinspires.ftc.teamcode.Matei.altu;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//
//import java.util.List;
//
//@Autonomous(name="TensorFlow", group="Matei")
//public class TensorFlow extends LinearOpMode {
//
//    private static final String KEY =
//            "ARhzqPT/////AAABmcks6V9uRE/4vJE+8qBUvnsYPXUfYlpJ8y+pzVN/GpzCrJsVanetvGKZxaJMs+3LmpTosqzKWHhdAiOzqd3kFmr4WYOWRErWkQuuVRx5/merGbBTYOAKQ9rkri+O3XR/l3bWk3zVlXUH7wXisifJcM2xoXGON4lYuETqenXu4NFfqOXkDGWI1nBNMM1dFW6AhLEuGt0R1TP6ToWiA1rk6dBvg7W3jGDi7eGYdvQhuo5I+6/ffn/OAyWnt+5DiJFVK365Cubaa0IE5xO3J4SSNcVXaho39lO5o7EhtCmqO2icWi8bYv7o+DHXWPsKfPByyrKSjEaXpvBNQ6S7P5pw9p5I5t6XafbS2LxYE5AJ6zH6";
//
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_FIRST_ELEMENT = "Quad";
//    private static final String LABEL_SECOND_ELEMENT = "Single";
//
//    private  VuforiaLocalizer vuforia;
//
//    private TFObjectDetector tfod;
//
//    @Override
//    public void runOpMode(){
//        initVuforia();
//        initTfod();
//
//        if(tfod != null){
//            tfod.activate();
//            tfod.setZoom(2.5, 16.0/9.0);
//        }
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                if (tfod != null) {
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//                        if (updatedRecognitions.size() == 0 ) {
//                            telemetry.addData("TFOD", "No items detected.");
//                            telemetry.addData("Target Zone", "A");
//                        } else {
//                            // list is not empty.
//                            // step through the list of recognitions and display boundary info.
//                            int i = 0;
//                            for (Recognition recognition : updatedRecognitions) {
////                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
////                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
////                                        recognition.getLeft(), recognition.getTop());
////                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
////                                        recognition.getRight(), recognition.getBottom());
//
////                                telemetry.addData(String.format("label plm"), recognition.getLabel());
////
//                                if (recognition.getLabel().equals("Single")) {
//                                    telemetry.addData("Target Zone", "B");
//                                } else if (recognition.getLabel().equals("Quad")) {
//                                    telemetry.addData("Target Zone", "C");
//                                } else {
//                                    telemetry.addData("Target Zone", "UNKNOWN");
//                                }
//                            }
//                        }
//
//                        telemetry.update();
//                    }
//                }
//            }
//        }
//    }
//
//    private void initVuforia(){
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = KEY;
////        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "web");
////        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//    }
//
//    private void initTfod(){
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorView", "id",  hardwareMap.appContext.getPackageName());
//
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//
//        tfodParameters.minResultConfidence = 0.8f;
//
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
//    }
//
//}
