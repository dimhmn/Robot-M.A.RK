#define BPG 2
#define BPD 3

// variables will change:
int buttonStateG = 0;         // variable for reading the pushbutton status
int buttonStateD = 0;         // variable for reading the pushbutton status

void setup() {
  Serial.begin(9600);
  // initialize the LED pin as an output:
  pinMode(BPG, INPUT_PULLUP);
  // initialize the pushbutton pin as an input:
  pinMode(BPD, INPUT_PULLUP);
}

void loop() {
  // read the state of the pushbutton value:
  buttonStateG = digitalRead(BPG);  
  buttonStateD = digitalRead(BPD);

  Serial.println(buttonStateG);
  Serial.println(buttonStateD);
}
