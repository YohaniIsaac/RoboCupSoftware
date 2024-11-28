#define boton1 3
#define boton2 10

void setup() {
  Serial.begin(9600);
  pinMode(boton1, INPUT_PULLUP);
  pinMode(boton2, INPUT_PULLUP);
}

void loop() {
  int buttonState1 = digitalRead(boton1);
  int buttonState2 = digitalRead(boton2);

  Serial.println(buttonState1);
  Serial.println(buttonState2);
  delay(100);
}
