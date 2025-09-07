# NEDrobotics – WRO Future Engineers 2025

This repository documents the work of **Team NEDrobotics** for the **World Robot Olympiad (WRO) 2025 Future Engineers** category.  
Our team consists of three members:

- Nicat Vəliyev  
- Elnur Məmmədov  
- Davud Məmmədov 

We designed, built, and programmed an autonomous vehicle capable of competing in the WRO Future Engineers challenges.  
This README explains our hardware, electronics, software modules, and the process to build and upload code to our robot.  
The goal is to make our work **clear, reproducible, and useful for other teams.**
<p> 
  <a href="https://www.labcenter.com" target="_blank">
    <img src="https://img.shields.io/badge/For%20Schematics-1032CF?logo=proteus&style=for-the-badge" />
  </a>

  <a href="https://www.arduino.cc/en/software" target="_blank">
    <img src="https://img.shields.io/badge/For%20Coding-00979D?logo=arduino&style=for-the-badge" />
  </a>

  <a href="https://www.youtube.com/@NEDRobotics" target="_blank">
    <img src="https://img.shields.io/badge/Youtube%20Channel-ff0000?logo=youtube&style=for-the-badge" />
  </a>
</p>

---

## Contents
1. Project Overview  
2. Hardware and Chassis Design  
3. Electronics and Components  
   - Power Management  
   - Sensors and Perception  
   - Actuators and Motor  
4. Wiring and Integration
5. Software and Control Architecture  
   - Arduino Code (Low-Level Control)  
   - Vision and AI Processing  
   - Obstacle and Race Management  
6. Bill of Materials 
7. Photos and Media  
8. Changes and Improvements  
9. Reproducibility and Future Work  

---

## 1. Project Overview
Team NEDrobotics joined the WRO Future Engineers challenge with the goal of creating a reliable and intelligent self-driving robot.  
The competition requires designing a car that can:

- Drive autonomously on a race track  
- Detect and avoid obstacles  
- Handle turns, walls, and markers  
- Demonstrate efficient use of sensors and algorithms  

Our design integrates **Arduino Mega 2560** for control, **HuskyLens Pro camera** for color recognition of traffic signs, **ultrasonic sensors** for distance detection from various targets.  
We combined **3D-printed parts, 12V DC motor, and servo-based steering** to create a modular, flexible and upgradable robot.

---

## 2. Hardware and Chassis Design
- **Chassis:** Built with 3D-printed parts for structure and flexibility  
- **Custom 3D Parts:** Hold electronics, motor, and sensors in place  
- **Motor:** One 12V DC motor for precise speed and torque control
- **Drive train:** Differential drive and steering system with LEGO parts
- **Steering:** One MG90 180° servo motor with Lego gears
- **Stability:** Low center of gravity increases balance during turns  

The final build is **strong, lightweight, reliable, and modular**.

---

## 3. Electronics and Components

### 3.1 Power Management
- **2S Li-ion 18650 Battery (7.4V):** Main power source  
- **LM2596 Converters (x3):** 5V for logic, servos, sensors
- **XL6009 Step up boost converter:** 12v for motor

 Provides stable power for each module  

### 3.2 Sensors and Perception
- **HuskyLens Pro AI Camera (x1):** Color recognition  
- **Ultrasonic Sensors (x3):** Distance detection  

### 3.3 Actuators and Motors

- **12V DC Motor (x1):** Drive system  
- **Servo Motor MG90 (x1):** Steering  
- **L298N Motor Driver (x1):** Motor interface with Arduino  

---

## 4. Wiring and Integration
- **Ultrasonic sensors → digital pins**  
- **Motor → L298N with PWM pins**  
- **Servo → dedicated PWM pins**  
- **HuskyLens Pro → Serial/I2C** 
###### ↓ Modules added to named regulators ↓
- **Ultrasonic sensors → LM2596 - A** 
- **Servo →  LM2596 - B**
- **HuskyLens Pro → LM2596 - C**  
###### ↑ Modules added to named regulators ↑
Power distributed via **LM2596 regulators** for stable voltage.

---

## 5. Software and Control Architecture

### 5.1 Arduino Code (Low-Level Control)
- Runs on Arduino Mega  
- Handles motor control, servo steering, ultrasonic readings 
- Uses PWM and smooth acceleration  

### 5.2 Vision and AI Processing
- Managed by HuskyLens Pro  
- Detects colors of the traffic signs  
- Sends data to Arduino for decisions  

### 5.3 Obstacle and Race Management
- State-machine logic with behaviors:  
  - START  
  - DETERMINE DIRECTION
  - EXIT PARKING LOT
  - REPEAT UNTIL 3 LAPS COMPLETED:
    - DETECT COLOR OF THE TRAFFIC SIGN
    - TURN LEFT AND FOLLOW LEFT WALL / TURN RIGHT AND FOLLOW RIGHT WALL
    - DETECT COLOR OF THE NEXT TRAFFIC SIGN AND MAKE SUITABLE CORNER TURN
  - PARK
  - FINISH  

### 5.4 The robot's operating logic and code sequence
 - **HuskyLens**
   - The Huskylens camera sends the colors of traffic signs it sees in the distance to the Arduino as raw data and colorID, and the Arduino processes them, enabling the robot to maneuver right or left.
 - **Ultrasonic sensors**
   - The ultrasonic sensors on the right and left sides determine how far away the robot is from the wall and in which direction it should move. During the loop, if the robot is moving clockwise, it uses the data from the right sensor to orient itself; if it is moving counterclockwise, it uses the data from the left sensor. When it's time to turn, if the sensor suddenly detects a very large distance, it starts to turn, stops after turning, checks the distance to the adjacent wall, and then realigns itself to move to the next turning point. *The front sensor only checks the distance to the parking walls during parking time, enabling it to park in an orderly manner.*

 - **Motor driver**
   - The L296N motor driver allows us to control the motor's speed and direction and adjust the brightness of the LEDs.

 - **Button**
   - The button allows us to run the code.

 - **LEDs**
   - The LEDs we placed on the front side assist the huskylens, making the colors of traffic signs appear more *vivid* and easier to see.

 - **Conventors and capacitors**
   - The LM2596 regulators reduce the voltage of the power coming from the battery and send it to the sensors/husklens and other components. *The capacitors supply the sudden current surge.*
### 5.5 Pseudo code
---

## 6. Bill of Materials

| Amount | Component | Notes | Prices |
|--------|-----------|-------|--------|
| 1 | [Arduino Mega 2560](other/arduinomega.jpg) | Main controller | <abbr title="10.7 USD (September 2025)">18.2₼</abbr> |
| 4 | [LM2596 buck converters](other/lm2596.jpg) | Voltage regulation | <abbr title="1.52 USD (September 2025)">2.6₼</abbr> |
| 1 | [L298N motor driver](other/l296n.jpg) | Motor control | <abbr title="2.05 USD (September 2025)">3.5₼</abbr> |
| 1 | [12V DC Motor](other/dc.jpg) | Drive | Free* |
| 3 | [Ultrasonic sensors](other/ultrasonic.jpg) | Distance measurement | <abbr title="4.7 USD (September 2025)">8₼</abbr> |
| 1 | [HuskyLens Pro AI camera](other/huskylens.jpg) | Vision | Free* |
| 1 | [MG90 servo motor (180°)](other/mg90.jpg) | Steering | <abbr title="2.64 USD (September 2025)">4.5₼</abbr> |
| 4 | [Li-ion 18650 Battery](other/li-ion.jpg) | Power source | <abbr title="11.17 USD (September 2025)">19₼</abbr> |
| 1 | [2x18650 Battery Holder](other/holder.jpg) | Misc | <abbr title="2.52 USD (September 2025)">4.3₼</abbr> | 
| 1 | [Lithium Batter charger](other/charger.jpg) | Misc | <abbr title="8.17 USD (September 2025)">13.9₼</abbr> |
| 3 | [LEDs](other/light.jpg) | Misc | Free* |
| 1 | [16v 4700µ Capacitor](other/16v.jpg) | Misc | <abbr title="1.76 USD (September 2025)">3₼</abbr> |
| 2 | [25v 4700µ Capacitors](other/25v.jpg) | Misc | <abbr title="2.05 USD (September 2025)">3.5₼</abbr> |
| Various | Wires, connectors, 3D-printed parts, Lego chassis | Assembly | Free* |

## **Total:** --- **<abbr title="49.4 USD (September 2025)">84₼</abbr>**

##### We paid <abbr title="41.1 USD (September 2025)">70₼</abbr> for the map, and we already had the boards for the walls, which we painted with black spray paint.
###### Items marked with “Free*” are parts that we had in previous races.

---

## 7. Photos and Media
### The robot's mission rounds *( /videos )*
<p> 
  <a href="https://youtube.com/shorts/DDyiXAjG3tE?feature=share" target="_blank">
    <img src="https://img.shields.io/badge/Open%20Challenge-ff0000?logo=youtube&style=for-the-badge" />
  </a>

  <a href="---" target="_blank">
    <img src="https://img.shields.io/badge/Obstacle%20Challenge-ff0000?logo=youtube&style=for-the-badge" />
  </a>
</p>

### Photos of the robot *( /v-photos )*

![The **top** of the robot](v-photos/top.jpeg)  
![The **bottom** of the robot](v-photos/bottom.jpeg)
![The **left side** of the robot](v-photos/left.jpeg)    
![The **right side** of the robot](v-photos/right.jpeg)  
![The **front** of the robot](v-photos/front.jpeg)
![The **back** of the robot](v-photos/back.jpeg)   

### The robot's electronic schematic  *( /schemes )*
![The **scheme** of the robot](schemes/scheme.png) 

### Photos of the team *( /t-photos )*
![Team photo](t-photos/team.jpeg) 
![Funny photo](t-photos/funny.jpeg) 

---

## 8. Changes and Improvements
During development we made important modifications:  
- Since the color sensor did not work very well and **the values it provided could not be very accurate**, we decided to remove the **TCS3200** from the robot. 
- Although the distance between the robot and the wall was small, the ultrasonic sensors perceived the distance as **far** because they were facing the wall **at an angle** slightly below it. This caused the robot to approach the wall **even closer**, leading to errors. While searching for a practical solution in the robot's driving algorithm, we added various automation algorithms such as the **PID** driving algorithm. Finally, to further improve performance, we decided to place the ultrasonic sensors **on the front of the robot.**
- We updated our HuskyLens camera with *HUSKYLENSWithModelV0.5.1bNorm* so it can better distinguish object colors and operate with fewer errors. The newer version slightly improved color recognition performance on the robot.
- We made the robot **rear-wheel drive** to **minimize the friction force** on the wheels and make it more **stable** and **powerful**. 
- Previously, there were no LEDs in front of the robot. Later, through experience, we realized that **it couldn't fully identify the colors of traffic signs**, so we placed three powerful **LEDs** on the front.
- Our old capacitors were **16V 1500µF (x1) and 16V 1000µF (x2)**. We later realized that these were not enough. We replaced the old capacitors with new **16V 4700µ (x1) and 25V 4700µ (x2)** capacitors. The new ones provided more enough power for the motor.
---
## 9. Reproducibility and Future Work
We aim to make our robot easy to reproduce.  
This repository includes source code, wiring diagrams, and 3D files.  

Future improvements:  
- Upgrade motor for higher speed  
- Use more advanced **AI camera system**
- Improve overall performance quality
- Use more **precise** sensors
- **Shortening** the robot's sizes 
- Use more **powerful** and more **efficient** batteries
---
## 10. Technical specifications
 - **Dimensions:** 275mm (L) x 160mm (W) x 200mm (H)
 - **Weight:** 727g
 - **Maximum speed:** 0.5m/s
 - **Driving system:** Rear-wheel drive (RWD)
 - **Steering Torque:** 24Ncm

---
## Conclusion
Team NEDrobotics’ project shows how teamwork, electronics, and coding can solve the WRO Future Engineers challenge.  
We created a **reliable, modular, and efficient autonomous robot** with Arduino, HuskyLens, and Lego EV3.  
This repository is a **complete guide** for anyone who wants to understand, rebuild, or improve our design.
