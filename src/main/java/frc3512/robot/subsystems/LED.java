package frc3512.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    public final AddressableLED leds;
    public final AddressableLEDBuffer ledBuffer;
    private int m_rainbowFirstPixelHue = 1;

    public boolean flashLEDs = false;
    int ledRate = 500;
    int ledTimer = 0;
    boolean ledsOn = false;

    public LED() {
        leds = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(50);

        leds.setLength(ledBuffer.getLength());
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
          }
          // Increase by to make the rainbow "move"
          m_rainbowFirstPixelHue += 50;
          // Check bounds
          m_rainbowFirstPixelHue %= 180;

        // for (var i = 0; i < ledBuffer.getLength(); i++) {
        //     ledBuffer.setRGB(i, noteIsIn[0], noteIsIn[1], noteIsIn[2]);
        // }

        // for (var i =0; i < ledBuffer.getLength(); i++) {
        //     ledBuffer.setHSV(i, 110, 100, 100);
        // }

        leds.setData(ledBuffer);
        leds.start();
    } 

    public void ledGreen() {
        for(var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 0, 255, 0);
        }
        leds.setData(ledBuffer);
    }

    public void ledRed() {
        for(var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }
        leds.setData(ledBuffer);
    }

    public void ledBlue() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 255);
        }
        leds.setData(ledBuffer);
    }

    public void ledsOff() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        leds.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        if (flashLEDs) {
            ledTimer += 20;
            if (ledTimer >= ledRate && !ledsOn) {
                ledGreen();
                ledTimer = 0;
                ledsOn = true;
            }
            else if (ledTimer >= ledRate && ledsOn) {
                ledsOff();
                ledTimer = 0;
                ledsOn = false;
            }
        }
    }
}
