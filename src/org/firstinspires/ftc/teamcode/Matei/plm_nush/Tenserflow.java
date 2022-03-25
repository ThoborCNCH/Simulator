//package org.firstinspires.ftc.teamcode.ftc16072;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.sun.xml.internal.bind.v2.ClassFactory;
//
//import java.util.List;
//
//public class Tenserflow {
//    String Key =
//            "ARhzqPT/////AAABmcks6V9uRE/4vJE+8qBUvnsYPXUfYlpJ8y+pzVN/GpzCrJsVanetvGKZxaJMs+3LmpTosqzKWHhdAiOzqd3kFmr4WYOWRErWkQuuVRx5/merGbBTYOAKQ9rkri+O3XR/l3bWk3zVlXUH7wXisifJcM2xoXGON4lYuETqenXu4NFfqOXkDGWI1nBNMM1dFW6AhLEuGt0R1TP6ToWiA1rk6dBvg7W3jGDi7eGYdvQhuo5I+6/ffn/OAyWnt+5DiJFVK365Cubaa0IE5xO3J4SSNcVXaho39lO5o7EhtCmqO2icWi8bYv7o+DHXWPsKfPByyrKSjEaXpvBNQ6S7P5pw9p5I5t6XafbS2LxYE5AJ6zH6";
//
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_FIRST_ELEMENT = "Quad";
//    private static final String LABEL_SECOND_ELEMENT = "Single";
//
//    private VuforiaLocalizer vuforia;
//
//    private TFObjectDetector tfod;
//
//    public void initVuforia() {
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//    }
//
//    public void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorView", "id", hardwareMap.appContext.getPackageName());
//
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//
//        tfodParameters.minResultConfidence = 0.8f;
//
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
//
//        if (tfod != null) {
//            tfod.activate();
//            tfod.setZoom(2.5, 16.0 / 9.0);
//        }
//    }
//
//    public String detect() {
//        String mesaj = null;
//        if (tfod != null) {
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                if (updatedRecognitions.size() == 0) {
//                    mesaj = "A";
//                } else {
//                    for (Recognition recognition : updatedRecognitions) {
//                        if (recognition.getLabel().equals("Single")) {
//                            mesaj = "B";
//                        } else if (recognition.getLabel().equals("Quad")) {
//                            mesaj = "C";
//                        } else {
//                            mesaj = "cacaNuEVoie";
//                        }
//                    }
//                }
//            }
//        }
//        return mesaj;
//    }
//}
