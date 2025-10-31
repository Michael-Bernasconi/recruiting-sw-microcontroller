// Pin digitale a cui è collegato il sensore Hall
const int hallPin = 2;

void setup() {
  // Inizializza il pin come input
  pinMode(hallPin, INPUT);

  // Inizializza la seriale a 115200 baud
  Serial.begin(115200);
}

void loop() {
  // Legge lo stato del sensore
  int state = digitalRead(hallPin);

  if (state == HIGH) {
    Serial.println("Presente");
  } else {
    Serial.println("Assente");
  }

  // Attesa di mezzo secondo
  delay(500);
}
