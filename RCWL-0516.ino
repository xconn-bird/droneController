
int sensor = 10;  //RCWL - 0516 Input pin
bool detect = false;
int led = 13; // led output pin

long duration;
float distance;


void setup() {
  Serial.begin(9600);
  Serial.println("Starting…\n");
  pinMode (sensor, INPUT);
  pinMode (led, OUTPUT);
  
}

void loop() {

  detect = digitalRead(sensor);
  if(detect == true) {
  digitalWrite(led, HIGH);
  duration = pulseIn(sensor , HIGH);
  distance = duration * 0.034 / 2;
  Serial.println("Distance");
  Serial.print(distance);
  Serial.println(" m ");
}
  else {
  digitalWrite(led, LOW);
  
}
    
}


 
  
