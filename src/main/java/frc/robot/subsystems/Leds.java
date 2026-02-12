package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Leds extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;

    private boolean shouldCheckIfIntakeIsAboutToHappen = true;
    private double lastToggleTime = 0;
    private static final double BLINK_PERIOD = 0.2;

    // Flicker control
    private boolean isFlickering = false;
    private double flickerStartTime = 0.0;
    private static final double FLICKER_DURATION = 3.0; // seconds
    private boolean ledOn = false;

    public Leds() {
        leds = new AddressableLED(1);
        ledBuffer = new AddressableLEDBuffer(60);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();
    }

    private void setAllLEDs(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        leds.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        super.periodic();

        RobotState robotState = RobotState.getInstance();

        // Start flicker if event happens
        if (shouldCheckIfIntakeIsAboutToHappen) {
            if (robotState.isHubAboutToBeInactive(3)) {
                shouldCheckIfIntakeIsAboutToHappen = false;

                isFlickering = true;
                flickerStartTime = Timer.getFPGATimestamp(); // start time
            }
        } else {
            if (robotState.isHubActive()) {
                shouldCheckIfIntakeIsAboutToHappen = true;
            }
        }

        if (isFlickering) {
            double now = Timer.getFPGATimestamp();

            if (now - lastToggleTime >= BLINK_PERIOD) {
                System.out.println("SWAPPED FLICKERING");

                ledOn = !ledOn;
                lastToggleTime = now;

                if (ledOn) {
                    setAllLEDs(255,255,255);
                } else {
                    setAllLEDs(0,0,0);
                }
            }
        }

    }
}
