~~~c
const int LED = 9;
int cds = A0;
void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(cds, INPUT);
}

void loop() {
  cds = analogRead(A0);
  
  if(cds > 150) {
    analogWrite(LED, 10);
  }
    else if( cds < 120 && cds > 50){
      analogWrite(LED, 50);
    }
    else if (cds < 50){
      digitalWrite(LED, HIGH);
    }
    Serial.println(cds);
}
