float compteur=0;
float nb_tour;//mesurée
float nb_tour_min;//mesurée
unsigned long current_time=0;
unsigned long previous_time=0;
float Q = 0.1; // Bruit du modèle (process noise)
float R = 3.0; // Bruit de mesure (measurement noise)
float x = 303; // État estimé
float P = 0; // Erreur de covariance estimée
int16_t pos = 0;
unsigned long Te= 2000,duree_acquisition=10000;//période d'échantillonage et durée d'acquisition
void setup() {
  // Démarrez la communication série à une vitesse de 9600 bauds
  Serial.begin(9600);
  pinMode(12, OUTPUT); // Broche Arduino réservée pour le sens de rotation du moteur A
  pinMode(9, OUTPUT); // Broche Arduino réservée pour le freinage du moteur A
  pinMode(3,OUTPUT);// PWM
  pinMode(2,INPUT_PULLUP);// encodeur
  attachInterrupt(digitalPinToInterrupt(2),fencodeur, RISING);
  // Initialisez d'autres configurations de votre choix ici
  digitalWrite(12, HIGH); // Le moteur A tourne dans le sens normal
  digitalWrite(9, HIGH); // Activation du frein moteur A
  analogWrite(3,0); // Pas de vitesse pour le moteur A (PWM)


  int i = 0;  //pre set capteur
  while( i <= 40){
    Capteur();
    delay(10);
    i += 1;
  }


}

void loop() {
  current_time=millis();
  if (current_time-previous_time>duree_acquisition){
    digitalWrite(12, HIGH); // Le moteur A tourne dans le sens normal
    digitalWrite(9, HIGH); // Activation du frein moteur A
    analogWrite(3,0); // Pas de vitesse pour le moteur A (PWM)
    }
    // on lit le caractère reçu
    char receivedChar = Serial.read();

    // Exécutez une fonction en fonction du caractère reçu
    if (receivedChar=='a') {
      fonctionA();
      compteur=0;
      //delay(1000);
  }

  delay(Te);
  encodeur();
  Capteur();
  Serial.println(pos);

  // Le reste de votre code peut continuer à s'exécuter ici
}

void fonctionA() {
  Serial.println("Touche 'a' pressée");
  previous_time=millis();
  digitalWrite(9, LOW); // Désactivation du frein moteur A
  analogWrite(3,255);// ne pas mettre en dessous de 50!!!
}
void fencodeur(){//fonction d'interruption
  compteur++;
}
void Capteur(void){
  int16_t t = pulseIn(5, HIGH);//branché sur le pin 5
  if (t != 0 && t <= 1850)
    {
      int16_t d = (t - 1000) * 2 ; // Convertir la largeur d'impulsion en microsecondes en distance en millimètres 
      d += 27; // ajout d'un offset mesurer sur le système réel, erreur prise avec un mètre
      if (d < 0) { d = 0; } // Limiter la distance minimale à 0.
      float K = (P+Q) / ((P+Q) + R);// Calcul du gain de Kalman
      x = x + K * (d - x);// Mise à jour de l'état estimé et de l'erreur de covariance estimée
      P = (1 - K) * (P+Q);
      pos = x; //Position filtrée
    }
  }
void encodeur(void){
  nb_tour = compteur/(50*64/4);
  nb_tour_min=nb_tour*60*1000/((float)Te);
  Serial.println(nb_tour);//nombre de tour
  Serial.println(nb_tour_min);
  compteur=0;
}