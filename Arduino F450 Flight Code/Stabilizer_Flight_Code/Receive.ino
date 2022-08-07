 void receive() {
      receiver_values[0] = pulseIn (receiver_pins[0], HIGH);
      receiver_values[1] = map(pulseIn (receiver_pins[1], HIGH), 1000, 2000, -250, 250);
      receiver_values[2] = map(pulseIn (receiver_pins[2], HIGH), 1000, 2000, -250, 250);
      receiver_values[3] = map(pulseIn (receiver_pins[3], HIGH), 1000, 2000, -250, 250);
      receiver_values[4] = pulseIn (receiver_pins[4], HIGH);
      receiver_values[5] = pulseIn (receiver_pins[5], HIGH);

      t_s = (int) MAFT();
      r_s = (int) MAFR();
      p_s = (int) MAFP();
      y_s = (int) MAFY();

//      Serial.print(t_s);
//      Serial.print(", ");
//      Serial.print(r_s);
//      Serial.print(", ");
//      Serial.print(p_s);
//      Serial.print(", ");
//      Serial.println(y_s);

      //Serial.println(receiver_values[5]);
      arm_switch = receiver_values[5];

}
