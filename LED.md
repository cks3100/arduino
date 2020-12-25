~~~c
const int LED = 9;
int cds = A0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(cds, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

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

