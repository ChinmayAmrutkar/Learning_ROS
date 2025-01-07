# TurtleSim Letter Drawing Project

## Overview
This project demonstrates fundamental concepts of **ROS** (Robot Operating System) through the **TurtleSim** simulation. The script showcases the turtle drawing the letters **H** and **I** by utilizing ROS publishers, services, and motion commands.

---

## Project Objectives
1. Understand ROS concepts such as **nodes**, **topics**, **publishers**, **subscribers**, and **services**.
2. Use the TurtleSim simulation to apply these concepts in a visually interactive way.
3. Create a modular and reusable codebase for drawing shapes and letters using a turtle.

---

## Key Features
- **Service Integration**: Utilizes `/turtle1/set_pen` to control pen settings and `/turtle1/teleport_absolute` for precise turtle positioning.
- **Motion Control**: Employs `/turtle1/cmd_vel` to publish movement commands for drawing straight and curved lines.
- **Modular Functions**: Implements reusable functions for drawing vertical and horizontal lines, which are combined to create letters.
- **Color Customization**: Allows pen color and thickness customization.

---

## What I Learned
- **ROS Node Communication**: Gained experience in setting up and interacting with ROS nodes using publishers and services.
- **Service Calls**: Learned how to call ROS services and handle their responses effectively.
- **Code Modularity**: Designed modular functions to simplify the process of drawing different shapes and letters.
- **Debugging**: Improved debugging skills by handling errors such as unavailable services and improper parameter settings.

---

## Requirements
### Software:
- **ROS** (tested with ROS Noetic)
- **TurtleSim package**
- **Python**

### Dependencies:
Install the required ROS package:
```bash
sudo apt install ros-noetic-turtlesim
```

---

## Setup and Execution
### 1. Start the ROS Master
Run the following command to start the ROS core:
```bash
roscore
```

### 2. Launch TurtleSim
Start the TurtleSim node in a new terminal:
```bash
rosrun turtlesim turtlesim_node
```

### 3. Run the Python Script
Execute the script to draw the letters:
```bash
python3 turtle_draw.py
```

---

## Code Explanation

### Main Functionalities:
1. **`call_teleport_absolute_service(x, y, theta)`**:
   Teleports the turtle to the specified `(x, y)` position and sets its orientation (`theta`).

2. **`call_set_pen_service(r, g, b, width, off)`**:
   Configures the pen’s color, thickness, and on/off state.

3. **`draw_vertical_line(x, y, time, reverse=False)`**:
   Draws a vertical line starting from `(x, y)` for the given `time`. Moves backward if `reverse=True`.

4. **`draw_horizontal_line(x, y, time)`**:
   Draws a horizontal line starting from `(x, y)` for the specified duration.

5. **`write_H()`**:
   Combines line-drawing functions to create the letter "H".

6. **`write_I()`**:
   Combines line-drawing functions to create the letter "I".

### Final Step:
After drawing, the turtle teleports to a resting position, and the pen is turned off to avoid unintended marks.

---

## Expected Output
When executed, the TurtleSim window will display the turtle drawing the letters **H** and **I** sequentially with red lines. <br>
<br>
![TurtleSim_Letter_Drawing](https://github.com/user-attachments/assets/86514814-4f9a-409b-9902-5a01a1defe9b)

---

## Challenges Faced
- **Precision**: Ensuring accurate turtle positioning to draw clean and proportionate letters.
- **Error Handling**: Debugging issues such as unresponsive services or incorrect movement commands.

---

## Future Enhancements
- Expand the script to draw more letters or custom shapes.
- Add real-time monitoring using subscribers to verify the turtle’s position.
- Enable dynamic input to draw user-specified words or patterns.
- Integrate additional ROS packages for enhanced functionality, such as motion planning.

---

## Troubleshooting
### Issue: "Service not available"
Ensure the TurtleSim node is running before executing the script.

### Issue: "Permission denied"
Make sure the script has execute permissions:
```bash
chmod +x turtle_draw.py
```

### Issue: "Command not found"
Verify that ROS is sourced in your terminal:
```bash
source /opt/ros/noetic/setup.bash
```

---

## Conclusion
This project was a hands-on way to explore and understand Baic ROS concepts while working with the TurtleSim environment. It highlights the importance of modular design and service utilization in robotics applications.

---
