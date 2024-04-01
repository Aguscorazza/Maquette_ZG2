float compteur=0;//encodeur
float nb_tour=0;//mesurée
float erreur=0,erreur_prec=0,delta_erreur=0,nb_tour_min=0;//mesurée
unsigned long current_time=0,tps_duree_acq=0,tps_corr=0,previous_time_duree_acq=0,previous_time_acq=0,previous_time_enc=0,previous_time_corr=0,tps_acq=0;//variable pour la gestion du temps
float Q = 0.1; // Bruit du modèle (process noise)
float R = 3.0; // Bruit de mesure (measurement noise)
int moteur=0,_moteur=0;//Etat moteur
int vitesse_tour_min=0;//consigne
unsigned long duree_acquisition=5000,Te= 200;//duree enregistrement et période d'échantillonage
float x = 303; // État estimé
float P = 0; // Erreur de covariance estimée
int16_t pos = 0;//distance (capteur)
int state_acq=0;//Statut de l'acquisition
int state_mode_oscil=0;//Choix du mode d'acquisition
int completSimu=0;//1 si la simulation est finie
float kp = 0.00,ki = 0.00,kd = 0.00;//coefficients du correcteur
float ui=0,cde=0,up=0,ud=0;// Variables du correcteur
float values[9]; // Tableau pour stocker les valeurs décimales reçues
char buffer[20]; // Créer un tampon pour stocker la chaîne de caractères formatée
void setup() {
  Serial.begin(115200);//Fixe le débit de la liaison série
  pinMode(12, OUTPUT); // Broche Arduino réservée pour le sens de rotation du moteur A
  pinMode(9, OUTPUT); // Broche Arduino réservée pour le freinage du moteur A
  pinMode(3,OUTPUT);// PWM
  pinMode(2,INPUT_PULLUP);// Broche encodeur
  attachInterrupt(digitalPinToInterrupt(2),fencodeur, RISING);//Appel de la fonction fencodeur sur front-montant
  digitalWrite(12, HIGH); // Le moteur A tourne dans le sens normal
  digitalWrite(9, HIGH); // Activation du frein moteur A
  analogWrite(3,0); // Pas de vitesse pour le moteur A (PWM)
  int i = 0;  //pre set capteur
  while( i <= 20){
    Capteur();
    delay(10);
    i += 1;
  }
  }
void loop() {
  if (Serial.available()){ //si on reçoit une tramme
      lectureIHM(); //lecture de la trame envoyée depuis l'ihm
    }
  Mise_a_jour_tps();
  if(state_acq == 1){//Demande d'acquisition
    if(state_mode_oscil == 0){//Mode oscillation libre
        if (tps_corr>10){
          Capteur();//mesure de position
          previous_time_corr=current_time;
        }
    }
    else if(state_mode_oscil == 1){//Mode oscillation forcé
      if (moteur==0){//Moteur à l'arrêt
        Capteur();//mesure position
        alu_mot();//Démarre le moteur
        encodeur();//mesure vitesse
        envoiDonnees();
        previous_time_corr=current_time;
        previous_time_acq=current_time;
      }
      else{//Moteur est déjà démarré
        if(tps_corr>5){
          Capteur();
          encodeur();//mesure vitesse rot
          Correcteur();
        }}}
    if(duree_acquisition<tps_duree_acq){
      state_acq = 0;}
    else if(tps_acq>Te){
      envoiDonnees();
      previous_time_acq=current_time;
    }
  }
  else if(state_acq == 0){//Arrêt acquisition
    completSimu=1;
    delay(30);
    Capteur();//mesure position
    arr_mot();//arrête le moteur
    envoiDonnees();
    state_acq = 2;
  }
  else if(state_acq == 2){//Attente
    completSimu = 0;
    compteur=0;
    nb_tour_min=0;
    cde=0;
    ui=0;
    erreur_prec=0;
    previous_time_duree_acq=current_time;
    previous_time_corr=current_time;
    previous_time_acq=current_time;
    previous_time_enc=current_time;
    }
}

void alu_mot() {
  digitalWrite(9, LOW); // Désactivation du frein moteur A
  analogWrite(3,(int)cde);// ne pas mettre en dessous de 50!!!
  moteur=1;
}
void arr_mot(){
    digitalWrite(9, HIGH); // Activation du frein moteur A
    analogWrite(3,0); // Pas de vitesse pour le moteur A (PWM)
    moteur=0;
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
void envoiDonnees(void){
    sprintf(buffer, "T%luR%dP%dC%d", tps_duree_acq,(int)nb_tour_min, (int)pos, completSimu); // Formater les données dans la chaîne de caractères nb_tour_min
    Serial.println(buffer); // Envoyer la chaîne de caractères formatée via Serial.print()
  }
void lectureIHM(void){
  String message = Serial.readStringUntil('/'); // Lit la chaîne de caractères jusqu'au caractère '/'
  float numValues = toFloatArray(message, values, 9);//transforme les chaines de caractères en float
  //Mise à jour des paramètres en fonction des valeurs reçues
  Te = (unsigned long)values[0];//période d'échantillonage
  state_acq = (int)values[1];//statut d'acquisition
  state_mode_oscil = (int)values[2];//Mode d'aquisition
  _moteur = (int)values[3];
  kp = values[4];
  ki = values[5];
  kd = values[6];
  vitesse_tour_min = (int)values[7];
  cde=(float)vitesse_tour_min/180*255;
  duree_acquisition = (unsigned long)values[8]*1000;
}
void encodeur(void){
  current_time=micros();//met à jour le temps courant pour plus de précision
  nb_tour_min=compteur/(50*64/4)*60*1000000/((float)(current_time-previous_time_enc));
  previous_time_enc=micros();//
  compteur=0;
}
int toFloatArray(String str, float arr[], int size) {
  int count = 0; // Compteur pour le nombre de valeurs extraites
  char* token = strtok(const_cast<char*>(str.c_str()), ","); // Divise la chaîne de caractères en sous-chaînes
  while (token != NULL && count < size) {
    arr[count++] = atof(token); // Convertit chaque sous-chaîne en nombre décimal et l'ajoute au tableau
    token = strtok(NULL, ",");
  }
  return count; // Retourne le nombre de valeurs extraites
}
void Correcteur(void){
  erreur=(float)vitesse_tour_min-nb_tour_min;
  up=kp*erreur;
  ui=ui+ki*erreur*5;//corrige la vitesse toutes les t=10ms
  delta_erreur=erreur-erreur_prec;
  ud=delta_erreur*kd/5;
  erreur_prec=erreur;
  cde=ui+up+ud;
  if(cde>=0){
    digitalWrite(12, HIGH);//tourne dans le sens normal
    if(cde>255){
      cde=255;
    }
  }
  else if(cde<0){
    cde=abs(cde);
    digitalWrite(12, LOW);//tourne dans l'autre sens
    if(cde>255){
      cde=255;
    }
  }
  analogWrite(3,(int)cde);//cde doit être entre 0 et 255
  previous_time_corr=millis();
  }
  void Mise_a_jour_tps(void){
    current_time=millis();
    tps_duree_acq=current_time-previous_time_duree_acq;
    tps_corr=current_time-previous_time_corr;
    tps_acq=current_time-previous_time_acq;
  }