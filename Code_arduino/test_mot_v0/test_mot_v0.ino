void setup() {// fonction pour initialiser les pins
  // Démarrez la communication série à une vitesse de 9600 bauds
  Serial.begin(9600);
  pinMode(12, OUTPUT); // Broche Arduino réservée pour le sens de rotation du moteur A
  pinMode(9, OUTPUT); // Broche Arduino réservée pour le freinage du moteur A
  pinMode(3,OUTPUT);// PWM pour contrôler la vitesse du moteur (0 à 255)
}

void loop() {
  digitalWrite(12, HIGH); // Le moteur A tourne dans le sens normal
  digitalWrite(9, HIGH); // Activation du frein moteur A (HIGH)
  analogWrite(3,0); // Pas de vitesse pour le moteur A (PWM)

  // Vérifiez si des données sont disponibles sur le port série
    // On lit le caractère reçu
    char receivedChar = Serial.read();

    //On exécute la fonction a si on a reçu le caractère 'a'
    if (receivedChar=='a') {
      fonctionA();
      delay(2000);
  }
  delay(50);
}

void fonctionA() {
  Serial.println("Touche 'a' pressée");// affiche du message
  digitalWrite(9, LOW); // Désactivation du frein moteur A
  analogWrite(3,100);// ne pas mettre en dessous de 50 ()
}
