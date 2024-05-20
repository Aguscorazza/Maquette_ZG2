volatile int compteur = 0;
float nb_tour;
float nb_tour_min;
unsigned long current_time=0;
unsigned long previous_time=0;
void setup() {
  // Démarrez la communication série à une vitesse de 9600 bauds
  Serial.begin(9600);
  pinMode(12, OUTPUT); // Broche Arduino réservée pour le sens de rotation du moteur A
  pinMode(9, OUTPUT); // Broche Arduino réservée pour le freinage du moteur A
  pinMode(3,OUTPUT);// PWM
  pinMode(2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2),fencodeur, RISING);
  // Initialisez d'autres configurations de votre choix ici
  digitalWrite(12, HIGH); // Le moteur A tourne dans le sens normal
  digitalWrite(9, HIGH); // Activation du frein moteur A
  analogWrite(3,0); // Pas de vitesse pour le moteur A (PWM)

}

void loop() {
  current_time=millis();
  if (current_time-previous_time>5000){
    digitalWrite(12, HIGH); // Le moteur A tourne dans le sens normal
    digitalWrite(9, HIGH); // Activation du frein moteur A
    analogWrite(3,0); // Pas de vitesse pour le moteur A (PWM)
    }

  // Vérifiez si des données sont disponibles sur le port série
    // Lisez le caractère reçu
    char receivedChar = Serial.read();

    // Exécutez une fonction en fonction du caractère reçu
    if (receivedChar=='a') {
      fonctionA();
      compteur=0;
      //delay(1000);
  }

  delay(200);
  nb_tour = (float)compteur/(50*64/4);
  nb_tour_min=nb_tour*60/0.2;
  //Serial.println(nb_tour);//nombre de tour
  Serial.println(nb_tour_min);
  compteur=0;


  // Le reste de votre code peut continuer à s'exécuter ici
}

void fonctionA() {
  Serial.println("Touche 'a' pressée");
  previous_time=millis();
  digitalWrite(9, LOW); // Désactivation du frein moteur A
  analogWrite(3,1000);// ne pas mettre en dessous de 50!!!
}
void fencodeur(){
  compteur = compteur + 1;
}

// Ajoutez d'autres fonctions pour d'autres caractères si nécessaire
