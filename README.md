# Quadcopter_Project_Simulink
This is a Simulink model to study quadcopter dynamics and PID controllers.

Gains were not automatically tuned since the model was not completely linearized.
The model was simplified by assuming near-hovering motion.
  roll, pitch in a range of -10 ~ +10 degrees (saturation block)
  roll, pitch rate was not saturated - to be adjusted later
  yaw was not assumed zero

Only 6 PID controllers (x, y, z, roll, pitch, yaw) were implemented

# TODO
1. Set saturation block for roll, pitch, yaw rates and re-adjust PID controllers accordingly
2. Implement GPS and IMU sensors (with random noise)
3. Implement Filter (e.g. Kalman Filter or LPF)
4. Make new plant model based on state-space form (Implemnet Kalman Filter as well)
5. Combine with various algorithms such as path-planning or obstacle avoidance 


# Contact:
owengun@dgist.ac.kr /n
Daegu Gyeongbuk Institute of Science and Technology
Republic of Korea
