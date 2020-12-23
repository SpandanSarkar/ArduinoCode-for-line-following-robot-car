int val=150;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
analogWrite(9,val);
analogWrite(10,0);
delay(1000);
analogWrite(9,val);
analogWrite(10,0);
delay(1000);
}
