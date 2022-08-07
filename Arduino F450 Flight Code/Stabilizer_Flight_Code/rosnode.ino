/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

void logging() 
{
  log_msg.roll = kal_roll;
  log_msg.pitch = kal_pitch;
  log_msg.control_roll = control_r_s;
  log_msg.control_pitch = control_p_s;
  log_msg.m1 = m1;
  log_msg.m2 = m2;
  log_msg.m3 = m3;
  log_msg.m4 = m4;
  log_msg.time = (t-t_start)/1000;
  
  logger.publish( &log_msg );
  nh.spinOnce();
//  delay(1000);
}
