package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.*;
import java.util.Map;
import java.util.*;

// WIP - Smartdash usage + livestream

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private static final int kPort = 0;
    private static final int kLength = 110;
  

  // all hues at maximum saturation and half brightness

  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);


  // Our LED strip has a density of 120 LEDs per meter

  private static final Distance kLedSpacing = Meters.of(1 / 120.0);


  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed

  // of 1 meter per second.

  private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), kLedSpacing);

    public LEDSubsystem() {
        
m_led = new AddressableLED(kPort); // PWM port 0
m_ledBuffer = new AddressableLEDBuffer(kLength); // Adjust the length according to your LED strip

m_led.setLength(m_ledBuffer.getLength());

// Set the data

m_led.setData(m_ledBuffer);

m_led.start();

// Create the buffer
AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(120);

// Create the view for the section of the strip on the left side of the robot.
// This section spans LEDs from index 0 through index 59, inclusive.
AddressableLEDBufferView m_left = m_buffer.createView(0, 55);

// The section of the strip on the right side of the robot.
// This view is reversed to cancel out the serpentine arrangement of the
// physical LED strip on the robot.

AddressableLEDBufferView m_right = m_buffer.createView(56, 110).reversed();
    
setDefaultCommand(runPattern(LEDPattern.solid(Color.kWhite)).withName("Off"));
    }

@Override
public void periodic() {
  // Periodically send the latest LED color data to the LED strip for it to display
  //m_scrollingRainbow.applyTo(m_ledBuffer);
m_led.setData(m_ledBuffer);
}

/**
 * Creates a command that runs a pattern on the entire LED strip.
 *
 * @param pattern the LED pattern to run
 */
public Command runPattern(LEDPattern pattern) {
  return run(() -> pattern.applyTo(m_ledBuffer));
}


public void scrollingRainbow(){
 // Update the buffer with the rainbow animation
 m_scrollingRainbow.applyTo(m_ledBuffer);
 // Set the LEDs
 LEDPattern dim = m_scrollingRainbow.atBrightness(Percent.of(20));
dim.applyTo(m_ledBuffer);

 m_led.setData(m_ledBuffer);

 m_led.start();
 
}
 
public void discontGradientRedBlue(){
// Create an LED pattern that displays a red-to-blue gradient.
// The LED strip will be red at one end and blue at the other.
LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
// Apply the LED pattern to the data buffer
gradient.applyTo(m_ledBuffer);
// Write the data to the LED strip
m_led.setData(m_ledBuffer);
}

public void continiousGradientRedBlue(){
// Create an LED pattern that displays a red-to-blue gradient.
// The LED strip will be red at one end and blue at the other.
LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue);
// Apply the LED pattern to the data buffer
gradient.applyTo(m_ledBuffer);
// Write the data to the LED strip
m_led.setData(m_ledBuffer);
}

public void steppedRedBlue(){
// Create an LED pattern that displays the first half of a strip as solid red,
// and the second half of the strip as solid blue.
LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));
// Apply the LED pattern to the data buffer
steps.applyTo(m_ledBuffer);
// Write the data to the LED strip
m_led.setData(m_ledBuffer);
}

public void progressWhite(double progressValue, double maxProgress){
// Create an LED pattern that displays a black-and-white mask that displays the current height of an elevator
// mechanism. This can be combined with other patterns to change the displayed color to something other than white.
LEDPattern pattern = LEDPattern.progressMaskLayer(() -> progressValue/maxProgress);
// Apply the LED pattern to the data buffer
pattern.applyTo(m_ledBuffer);
// Write the data to the LED strip
m_led.setData(m_ledBuffer);
}

///* THIS SECTION DEMONSTRATES MODIFIER FUNCTIONS *///

public void demo(){
    // Create an LED pattern that displays a red-to-blue gradient, offset 40 pixels forward.
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
    LEDPattern pattern = gradient.offsetBy(40);
    LEDPattern negative = gradient.offsetBy(-20); // Equivalent to the above when applied to a 60-LED buffer
    // Apply the LED pattern to the data buffer
    pattern.applyTo(m_ledBuffer);
    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
}

public void demo2(){
    // Create an LED pattern that displays a red-to-blue gradient, then reverse it so it displays blue-to-red.
LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
LEDPattern pattern = base.reversed();
// Apply the LED pattern to the data buffer
pattern.applyTo(m_ledBuffer);
// Write the data to the LED strip
m_led.setData(m_ledBuffer);
}

public void demo3(){
// Create an LED pattern that displays a red-to-blue gradient, then scroll at one quarter of the LED strip's length per second.
// For a half-meter length of a 120 LED-per-meter strip, this is equivalent to scrolling at 12.5 centimeters per second.
Distance ledSpacing = Meters.of(1 / 110.0);
LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kCyan, Color.kCoral);
LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(200));
LEDPattern absolute = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(100), ledSpacing);
// Apply the LED pattern to the data buffer
// pattern.applyTo(m_ledBuffer);
// Write the data to the LED strip

LEDPattern pattern2 = absolute.atBrightness(Percent.of(20));
pattern2.applyTo(m_ledBuffer);

m_led.setData(m_ledBuffer);
}

public void demo4(){
// Create an LED pattern that displays a red-to-blue gradient, breathing at a 2 second period (0.5 Hz)
LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
LEDPattern pattern = base.breathe(Seconds.of(2));
// Apply the LED pattern to the data buffer
pattern.applyTo(m_ledBuffer);

// Write the data to the LED strip
m_led.setData(m_ledBuffer);
}

public void demo5(){
// Create an LED pattern that displays a red-to-blue gradient, blinking at various rates.
LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);

// 1.5 seconds on, 1.5 seconds off, for a total period of 3 seconds
LEDPattern pattern = base.blink(Seconds.of(1.5));

// 2 seconds on, 1 second off, for a total period of 3 seconds
LEDPattern asymmetric = base.blink(Seconds.of(2), Seconds.of(1));

// Turn the base pattern on when the RSL is on, and off when the RSL is off
LEDPattern sycned = base.synchronizedBlink(RobotController::getRSLState);

// Apply the LED pattern to the data buffer
pattern.applyTo(m_ledBuffer);

// Write the data to the LED strip
m_led.setData(m_ledBuffer);
}



public void demo6(){
// Create an LED pattern that displays a red-to-blue gradient at half brightness
LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
LEDPattern pattern = base.atBrightness(Percent.of(50));
// Apply the LED pattern to the data buffer
pattern.applyTo(m_ledBuffer);
// Write the data to the LED strip
m_led.setData(m_ledBuffer);
}

/* COMBANITORY DEMOS OF FEATURES + MODIFIERS */

public void gradientProgress(ElevatorSubsystem elevator) {
    try {
        // Get the current position of the elevator
        double position = elevator.getPosition();

        // Ensure the elevator position is within expected bounds
        double normalizedPosition = Math.max(0, Math.min(1, position / -60));

        // Create an LED pattern that displays a red-to-blue gradient at a variable length
        // depending on the relative position of the elevator. The blue end of the gradient
        // will only be shown when the elevator gets close to its maximum height; otherwise,
        // that end will be solid black when the elevator is at lower heights.
        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kMaroon, Color.kBlue);
        LEDPattern pattern = LEDPattern.progressMaskLayer(() -> normalizedPosition);
        LEDPattern heightDisplay = base.mask(pattern);

        // Apply the LED pattern to the data buffer
        heightDisplay.applyTo(m_ledBuffer);

        // Write the data to the LED strip
        m_led.setData(m_ledBuffer);

        // Debug information
        System.out.println("Elevator Position: " + position);
        System.out.println("Normalized Position: " + normalizedPosition);
    } catch (Exception e) {
        e.printStackTrace();
    
}}


/* UTILIZED METHODS / REAL COMMANDS */

public void strobe(Color strobecolor, double strobespeed){
    // Create an LED pattern that displays the chosen color
    LEDPattern base = LEDPattern.solid(strobecolor);
    
    LEDPattern pattern = base.blink(Seconds.of(strobespeed));
    
    // Apply the LED pattern to the data buffer
    pattern.applyTo(m_ledBuffer);
    
    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);


    }

    public void progress(double progressValue, double maxProgress, Color ColorA, Color ColorB){
        // Create an LED pattern that displays a red-to-blue gradient at a variable length
        // depending on the relative position of the elevator. The blue end of the gradient
        // will only be shown when the elevator gets close to its maximum height; otherwise,
        // that end will be solid black when the elevator is at lower heights.
        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, ColorA, ColorB);
        LEDPattern pattern = LEDPattern.progressMaskLayer(() -> progressValue/maxProgress);
        LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));
        LEDPattern heightDisplay = base.mask(pattern);
        LEDPattern scrollDisplay = base.mask(heightDisplay);
        // Apply the LED pattern to the data buffer
        scrollDisplay.applyTo(m_ledBuffer);
        // Write the data to the LED strip
        m_led.setData(m_ledBuffer);
        }

        public void patriot(){
            // Create an LED pattern that displays a red-to-blue gradient, then scroll at one quarter of the LED strip's length per second.
            // For a half-meter length of a 120 LED-per-meter strip, this is equivalent to scrolling at 12.5 centimeters per second.
            Distance ledSpacing = Meters.of(1 / 110.0);
            LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kBlue, Color.kWhite);
          //  LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(200));
            LEDPattern absolute = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(200), ledSpacing);
            // Apply the LED pattern to the data buffer
            // pattern.applyTo(m_ledBuffer);
            // Write the data to the LED strip
            
            LEDPattern pattern2 = absolute.atBrightness(Percent.of(20));
            pattern2.applyTo(m_ledBuffer);
            
            m_led.setData(m_ledBuffer);
            }

            public void reefPose(Color ColorA, Color ColorB){
                Distance ledSpacing = Meters.of(1 / 110.0);
                // Create an LED pattern that displays a red-to-blue gradient, breathing at a 2 second period (0.5 Hz)
                LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, ColorA, ColorB);
              //  LEDPattern pattern = base.breathe(Seconds.of(0.2));
                LEDPattern blink = base.blink(Seconds.of(0.05));
                LEDPattern absolute = blink.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(300), ledSpacing);
                // Apply the LED pattern to the data buffer
                absolute.applyTo(m_ledBuffer);
                // Write the data to the LED strip
                m_led.setData(m_ledBuffer);
                }


        
}
