package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLightSubsystem extends SubsystemBase {
    private final AddressableLED addressableLED = new AddressableLED(7);
    private final AddressableLEDBuffer addressableLEDBuffer = new AddressableLEDBuffer(150);
    private int rainbowFirstPixelHue = 0;
    private LedState currentState = LedState.RED;

    public enum LedState {
        RED((ledBuffer) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 255, 0, 0);
            }
        }),
        BLUE((ledBuffer) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 0, 0, 255);
            }
        }),
        PURPLE((ledBuffer) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 145, 0, 255);
            }
        }),
        YELLOW((ledBuffer) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 255, 225, 0);
            }
        }),
        REDPULSE((ledBuffer) -> {
            // TODO: make it actually pulse like it's really angry
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 255, 0, 0);
            }
        }),
        BLUEPULSE((ledBuffer) -> {
            // TODO: make it actually pulse kinda like it's hurting
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 0, 0, 255);
            }
        }),
        RAINBOW((ledBuffer) -> {

        }),
        OFF((ledBuffer) -> {

        });

        public final Consumer<AddressableLEDBuffer> setBuffer;

        LedState(Consumer<AddressableLEDBuffer> bufferSetter) {
            setBuffer = bufferSetter;
        }
    }

    public LEDLightSubsystem() {
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()){
            currentState.setBuffer.accept(addressableLEDBuffer);
        }
        else{
            
        }
        addressableLED.setData(addressableLEDBuffer);
    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < addressableLEDBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / addressableLEDBuffer.getLength())) % 180;
            // Set the value
            addressableLEDBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }
}
