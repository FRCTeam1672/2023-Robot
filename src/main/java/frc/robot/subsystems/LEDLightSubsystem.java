package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLightSubsystem extends SubsystemBase{
    private final AddressableLED addressableLED = new AddressableLED(7);
    private final AddressableLEDBuffer addressableLEDBuffer = new AddressableLEDBuffer(150);
    private int rainbowFirstPixelHue = 0;

    public LEDLightSubsystem(){
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    @Override
    public void periodic() {
        rainbow();
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
