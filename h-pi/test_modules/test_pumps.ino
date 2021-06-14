const int pump1 = 14;
const int pump2 = 27;
const int pump3 = 26;
const int pump4 = 25;
int p1_delay = 10000; 
int p2_delay = 10000;
int p3_delay = 4000;
int p4_delay = 4000;
int pump_timeout = 0;
void setup()
{

  Serial.begin(115200);
  Serial.println(F("H-PI - Human-Plant Interface"));

  /*
  * Configure pump pins
  */
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(pump3, OUTPUT);
  pinMode(pump4, OUTPUT);

  /*
  * Misc.
  */
  pumps_off();
}

void pumps_off()
{
  digitalWrite(pump1, HIGH);
  digitalWrite(pump2, HIGH);
  digitalWrite(pump3, HIGH);
  digitalWrite(pump4, HIGH);
}

void pump_select(int32_t pump, int32_t pump_time, int* pump_timeout_val)
{
  Serial.println("Pumping")
  digitalWrite(pump, LOW);
  delay(pump_time);
  *pump_timeout_val = pump_timeout;
  pumps_off();
  Serial.println("Off")
}



void loop()
{
  Serial.println("Loop");
  delay(1000);
  pump_select(pump2, p2_delay, &pump_timeout);
}