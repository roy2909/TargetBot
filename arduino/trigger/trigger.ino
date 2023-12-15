/**
 * @file gun_control.ino
 * @brief Arduino code for controlling two guns through a serial interface.
 * @author Max
 * @date 12/1/2023
 */

/**
 * @struct gun_pins
 * @brief Structure to represent the pin configuration for a gun.
 */
struct gun_pins {
  int rev_pin;
  int fire_pin;
};

gun_pins gun0 = {2,3};
gun_pins gun1 = {4,5};

gun_pins guns[] = {gun0, gun1};

/**
 * @brief Setup function executed once when the Arduino starts.
 * Initializes serial communication and sets up pin modes.
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5,OUTPUT);
}

/**
 * @brief Main loop function executed continuously.
 * Listens for incoming serial data to control the guns.
 */
void loop() {
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    int shooting_gun = Serial.parseInt();
    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
      // reply to the node
      digitalWrite(guns[shooting_gun].rev_pin, HIGH);
      delay(1000);
      digitalWrite(guns[shooting_gun].fire_pin, HIGH);
      delay(1000);
      digitalWrite(guns[shooting_gun].fire_pin, LOW);
      digitalWrite(guns[shooting_gun].rev_pin, LOW);

      Serial.println(String(shooting_gun));
    }
  }

}
