#define eStop 4         // White Wire
#define redLED 5        // Yellow Wire
#define greenLED 6      // Green Wire
#define ledVcc 13       // Blue Wire

#define holdButtonTime 5000

// The red and green LED need to go LOW to be active

int currentButtonState = 0;
int previousButtonState = 0;
unsigned long buttonTimer = 0;
bool latchedState = false;

void setup() 
{
  Serial.begin(9600);
  pinMode(eStop, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(ledVcc, OUTPUT);
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW); // Default green is active (e-stop not in use)
  digitalWrite(ledVcc, HIGH);

}

void loop() 
{
  currentButtonState = digitalRead(eStop);
  currentButtonState = !currentButtonState;
  // When e-stop is pressed, redLED active
  if(currentButtonState == HIGH && previousButtonState == LOW)
  {
    buttonTimer = millis();
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, HIGH);
    Serial.println("STOP");
  }

  // Unlatch the estop button by holding it down for 5s
  if(currentButtonState == HIGH && holdButtonTime <= (millis() - buttonTimer))
  {
    latchedState = false;
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
    Serial.println("RUN");
  }

  if(currentButtonState == LOW && previousButtonState == HIGH)
  {
    latchedState = true;
  }

  previousButtonState = currentButtonState;

}

