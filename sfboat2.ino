int sp;  // speed
int st;  // steer
int lpump;
int rpump;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(52, INPUT);
  pinMode(50, INPUT);
  
  sp = pulseIn(52, HIGH );
  st = pulseIn(50, HIGH);
  lpump = 46;
  rpump = 48;
  pinMode(lpump, OUTPUT);
  pinMode(rpump, OUTPUT);

  digitalWrite(lpump, HIGH);
  digitalWrite(rpump, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

  sp = pulseIn(52, HIGH, 30000);
  st = pulseIn(50, HIGH, 30000);
Serial.println(sp);
Serial.println(st);
  /*
    Boundries:
    sp = 1500-2100
    st = 1700-2100,1300-900      */

  if ((sp > 1500) && (sp < 2100)) {       //Go forward
    
    if ((st > 1700) && (st < 2100)) {     //Right turn
      digitalWrite(lpump, LOW);
      digitalWrite(rpump, HIGH);
    }
    else if ((st > 900) && (st < 1300)) { //Left turn
      digitalWrite(lpump, HIGH);
      digitalWrite(rpump, LOW);
    }
    else {
      digitalWrite(lpump, LOW);
      digitalWrite(rpump, LOW);
    }

  }


  else {
    digitalWrite(lpump, HIGH);
    digitalWrite(rpump, HIGH);
  }

  //Serial.println(ch0);
  //Serial.println(ch1);
  delay(50);
}
