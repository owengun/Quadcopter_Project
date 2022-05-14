# Quadcopter_Project_Simulink
This is a Simulink model to study quadcopter dynamics and PID controllers.

Gains were not automatically tuned since the model was not completely linearized.
The model was simplified by assuming near-hovering motion.
  roll, pitch in a range of -10 ~ +10 degrees (saturation block)
  roll, pitch rate was not saturated - to be adjusted later
  yaw was not assumed zero

Only 6 PID controllers (x, y, z, roll, pitch, yaw) were implemented
