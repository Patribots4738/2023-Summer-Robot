package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import java.util.LinkedList;
import java.util.Queue;

public class ArduinoController extends SubsystemBase {
  //Sets up the Arduino over I2C on port 8
  private final I2C arduino;
  private Queue<Integer> queue;
  private int currentState = -1;

  public ArduinoController() {
    arduino = new I2C(I2C.Port.kOnboard, LEDConstants.ARDUINO_ADDRESS);
    queue = new LinkedList<Integer>();
  }

  @Override
  public void periodic() {
    // Write the latest byte in the queue to the arduino
    // If it exists
    if (queue.peek() != null) {
      // If the state is the same as the current state, don't send it
      // and remove it from the queue
      if (currentState == queue.peek()) {
        queue.remove();
        return;
      }
      // Set the current state to our value
      currentState = queue.peek();
      // Send the latest byte in the queue to the arduino
      sendByte();
    }
  }

  public void setLEDState(int state) {
    // Add the state to the queue
    if (!queue.contains(state) && state != currentState) {
      queue.offer(state);
    }
  }

  public void sendByte() {
    // Send the latest queue value to the arduino,
    // Then, remove the latest value from the queue
    if (queue.peek() != null) {
      // System.out.println("Sending byte: " + queue.peek());
      arduino.write(LEDConstants.ARDUINO_ADDRESS, queue.poll());
    }
    
  }
}