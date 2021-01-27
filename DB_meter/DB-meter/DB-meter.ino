int led= 13;     // 'led' is the Arduino onboard LED
int mic= A0;     // 'mic' is the Arduino pin A0 = the analog output pin of the Microphone board (A0)
int val = 0;     // variable to store the analog microphone value

void setup () 
{
  pinMode (led, OUTPUT);
  Serial.begin (9600);
}
 
void loop () 
{
  val = analogRead (mic);
//  delay (50);
  Serial.println (val, DEC);
}
