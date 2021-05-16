

const int pump1 = 14;
const int pump2 = 27;
const int pump3 = 26;
const int pump4 = 25;
const int moisture_sensor1 = 32;
const int moisture_sensor2 = 35;
const int moisture_sensor3 = 34;
const int push_button = 12;
int relay_flag = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("DHTxx test!"));
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(pump3, OUTPUT);
  pinMode(pump4, OUTPUT);
  pinMode(pump1, OUTPUT);

  pinMode(push_button, INPUT_PULLUP);

  pinMode(moisture_sensor1, INPUT);
  pinMode(moisture_sensor2, INPUT);
  pinMode(moisture_sensor3, INPUT);

//  attachInterrupt(digitalPinToInterrupt(push_button), handlePushButton, FALLING); // trigger when button pressed, but not when released.


}

void handlePushButton() {
  /* 
   *  Handle the interrupt for user pressing the push button
   */
   
  if (relay_flag == 0) {
    relay_flag = 1;
  } else {
    relay_flag = 0;
  }

}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("Run");
    Serial.println(digitalRead(push_button));
    Serial.println(analogRead(moisture_sensor1));
    Serial.println(analogRead(moisture_sensor2));
    Serial.println(analogRead(moisture_sensor3));


    if (digitalRead(push_button) == HIGH){
      digitalWrite(pump1, HIGH);
      digitalWrite(pump2, HIGH);
      digitalWrite(pump3, HIGH);
      digitalWrite(pump4, HIGH);
    }
    else if (digitalRead(push_button) == LOW){
      digitalWrite(pump1, LOW);
      delay(100);
      digitalWrite(pump2, LOW);
      delay(100);
      digitalWrite(pump3, LOW);
      delay(100);
      digitalWrite(pump4, LOW);
    }


}
