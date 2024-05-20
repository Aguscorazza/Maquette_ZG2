#include <util/atomic.h>

#define UART_BAUDRATE     38400
#define TX_BUFFER_SIZE    20
#define RX_BUFFER_SIZE    20

#define FREQ_TIM1         20        // [Hz] 
#define FREQ_PWM          25000     // [Hz]

#define MAX_TENSION       12    // V
#define ENCODER_CPR       64    // Encoder Counts per Revolution (CPR)
#define MOTOR_GEAR_RATIO  50
#define MAX_I_ERROR       1000
#define pi                3.1416

#define Kt 38.4615            // Motor Torque Constant
#define Ke (Kt)
#define Ra 2.2
#define La 0.0023
#define R 72.2566             // 2*PI*fc*La // fc = 5000hz
#define bm 0.36728            // No-load current * Kt / No-load speed 

// globals
long prevT = 0;
int posPrev = 0;
float v1Prev = 0;

volatile int pos_i=0;                     // Number of counts of the encoder signal (used to calcule motor speed)

typedef struct{
  int duration=0;                   // Simulation duration [s]
  int mode=0;                       // 0: Oscillation Libre - 1: Oscillation forcee
  float commanded_motor_speed=0.0;  // Motor speed (consigne) [rpm]
  
  float i_error=0;
  float Kp = 20.0;
  float Ki = 10.0;
  unsigned int Te = 50;                      // Période d'échantillonnage [ms]
  
  float measured_motor_speed=0.0;           // Motor Speed [rpm]
  float measured_motor_current=0.0;         // Motor current [A]
  int measured_position=0;                  // Position [m]
  bool simulation_running=false;
} Simulation;

char TX_buffer[TX_BUFFER_SIZE];
char RX_buffer[RX_BUFFER_SIZE];
char TEST_BUFFER[TX_BUFFER_SIZE];
volatile int rx_index=0, command_flag=0;

Simulation simulation;

void PWM_set_duty(int duty) {
  if(duty>255){
    duty = 255;
  }
  if(duty>=0){
    analogWrite(3, duty);
  }
}

void PIController(float setpoint, float deltaT){
  setpoint = setpoint * 2.0 * pi / 60.0; // [rpm]-->[rad/s]
  
  float speed_error = setpoint-simulation.measured_motor_speed;
  simulation.i_error += speed_error*deltaT; // Integral error
  
  // Anti-windup mechanism to limit the integral term
  if (simulation.i_error > MAX_I_ERROR) {
      simulation.i_error = MAX_I_ERROR;
  } else if (simulation.i_error < -MAX_I_ERROR) {
      simulation.i_error = -MAX_I_ERROR;
  }

  float control_torque = simulation.Kp * speed_error + simulation.Ki * simulation.i_error;
  float control_current = (control_torque + bm * simulation.measured_motor_speed)/Kt;
  float control_tension = R*(control_current - simulation.measured_motor_current) + Ke * simulation.measured_motor_speed + Ra * simulation.measured_motor_current;

  int output_dutycycle = (int)(control_tension * 100) / (MAX_TENSION);
  PWM_set_duty(output_dutycycle);
}

/**
 * Initialisation de l'UART (Universal Asynchronous Receiver-Transmitter) avec les paramètres spécifiés.
 * 
 * @param baud_rate Taux de bauds de communication.
 * @param intRx Activation de l'interruption de réception (1 pour activer, 0 pour désactiver).
 * @param intTx Activation de l'interruption de transmission (1 pour activer, 0 pour désactiver).
 * 
 * Cette fonction configure l'UART pour le fonctionnement souhaité avec les paramètres spécifiés.
 * Elle initialise le registre de bauds, active la réception et/ou la transmission selon les paramètres
 * fournis, et active les interruptions de réception et/ou de transmission si spécifiées.
 */
void UART_init(uint64_t baud_rate, uint8_t intRx, uint8_t intTx) {
  UBRR0 = F_CPU/16/baud_rate-1; // Configuration du registre de bauds
  UCSR0A &= ~(1<<U2X0); // Assure que le bit U2X0 est à 0 (vitesse normale)
  UCSR0B = (1<<RXEN0) | (1<<TXEN0); // Activation de la réception et de la transmission
  UCSR0C = (3<<UCSZ00); // Configuration des bits de stop et de la taille des données (1 bit de stop, 8 bits de données)
  
  if(intRx) {
    UCSR0A |= (1<<RXC0); // Désactive le drapeau de réception complète
    UCSR0B |= (1<<RXCIE0); // Activation de l'interruption en cas de réception complète
  }
  if(intTx){
    UCSR0A |= (1<<TXC0); // Désactive le drapeau de transmission complète
    UCSR0B |= (1<<TXCIE0); // Activation de l'interruption en cas de transmission complète
  }
}


void put_char(char c) {
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = c;
}

void put_string(char *string) {
  while(*string) {
    put_char(*string++);
  }
}

void get_command(const char *command){
  switch(command[0]){
    case 'S': //Start Simulation
      start_simulation_command(command, &simulation);
      break;
    case 'E':
      stop_simulation_command(command, &simulation);
      break;
    case 'P':
      set_parameters_command(command, &simulation);
      break;
    default:
      break;
  }
}

void start_simulation_command(const char *command, Simulation *simulation){
  sscanf(command, "S_%d_%d", &(simulation->mode), &(simulation->commanded_motor_speed));
  simulation->commanded_motor_speed = 2.0*pi*simulation->commanded_motor_speed/60.0; // [rpm]-->[rad/s]
  
  switch(simulation->mode){
    case 1: //Oscillation forcée      
      digitalWrite(9, LOW);
      break;
      
    case 0: // Oscillation libre
    default:
      digitalWrite(9,HIGH);
      PWM_set_duty(0);
      break;
  }   
  // Start simulation
  simulation->simulation_running=true;
}

void stop_simulation_command(const char *command, Simulation *simulation){
  simulation->simulation_running=false;
  // Arrêter le moteur
  PWM_set_duty(0);
}

int get_distance(){
  int t = pulseIn(8, HIGH);//branché sur le pin 5
  if (t != 0 && t <= 1850)
    {
      return (t - 1000) * 2 ; // Convertir la largeur d'impulsion en microsecondes en distance en millimètres 
    }
}

void set_parameters_command(const char *command, Simulation *simulation){
  sscanf(command, "P%d-I%d-T%d", &(simulation->Kp), &(simulation->Ki), &(simulation->Te));
}

void setup() {
  noInterrupts();                                               // Désactive les interruptions pendant la configuration initiale
  UART_init(UART_BAUDRATE, 1, 0);                               // Initialise la communication UART avec le taux de bauds spécifié
  // Configuration des broches
  pinMode(13, OUTPUT);                                          // Configure la broche 13 en sortie
  pinMode(12, OUTPUT);                                          // Broche Arduino réservée pour le sens de rotation du moteur A
  pinMode(9, OUTPUT);                                           // Broche Arduino réservée pour le freinage du moteur A
  pinMode(3, OUTPUT);                                           // Broche PWM pour le contrôle de la vitesse du moteur A
  pinMode(2, INPUT);                                            // Broche d'entrée pour l'encodeur avec résistance de pull-up activée
  attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING); // Associe la fonction d'interruption encoderISR à la broche d'encodeur, déclenchée sur front-montant

  // État initial des broches
  digitalWrite(12, HIGH); // Le moteur A tourne dans le sens normal
  digitalWrite(9, HIGH);  // Activation du frein moteur A
  analogWrite(3, 0);      // Pas de vitesse pour le moteur A (PWM)
  
  interrupts(); // Réactive les interruptions après la configuration initiale

  put_string("UART OK"); // Envoie un message indiquant que la configuration UART est terminée
}

void loop() {
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e3;
  if(simulation.simulation_running==true && deltaT > simulation.Te){  
    // Mesurer la distance de la masse
    simulation.measured_position = get_distance();
    
    // read the encoder counts in an atomic block
    // to avoid potential misreads
    int pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      pos = pos_i;
    }
    deltaT = deltaT/1.0e3;
    float velocity1 = (pos - posPrev)/deltaT;
    posPrev = pos;

    // Convert count/s to RPM
    float v1 = velocity1/800.0*60.0;
    // Low-pass filter (25 Hz cutoff)
    simulation.measured_motor_speed = (0.854*simulation.measured_motor_speed + 0.0728*v1 + 0.0728*v1Prev)*2.0*pi/60.0; // [rpm]-->[rad/s]
    v1Prev = v1;
    
    // Measure the motor current
    simulation.measured_motor_current = map(analogRead(A0), 0, 1024, 0, 5000)*2.0/(1000.0*3.3);
      
    if(simulation.mode==1){
      PIController(simulation.commanded_motor_speed, deltaT);
    }
    
    // Envoyer les données périodiquement
    sprintf(TX_buffer, "#P%d-V%d-I%d%", simulation.measured_position, simulation.measured_motor_speed, simulation.measured_motor_current);
    put_string(TX_buffer);
    prevT = currT;
  }

}

/**
 * Fonction d'interruption pour la gestion des compteurs d'encodeurs.
 * 
 * Cette fonction est appelée à chaque fois qu'une interruption est générée par l'encodeur.
 * Son rôle est d'incrémenter le compteur d'encodeurs pour suivre le nombre d'impulsions reçues.
 */
void encoderISR(){ 
  pos_i++; // Incrément du compteur de l'encodeur
}


/**
 * Routine de service d'interruption (ISR) déclenchée lors de la réception de données sur l'USART.
 * 
 * Cette ISR est activée lorsqu'un caractère est reçu sur le port de communication série (USART).
 * Elle gère la réception de commandes, délimitées par les caractères '#' et '%'. Elle stocke les
 * caractères reçus dans un tampon circulaire jusqu'à ce que la commande soit complète, puis elle
 * passe la commande complète à la fonction de traitement des commandes.
 */
 
ISR(USART_RX_vect){
  digitalWrite(13,HIGH);
  char RX_received_char; // Caractère reçu depuis l'USART
  RX_received_char = UDR0; // Lecture du caractère reçu
  
  // Initialiser le tampon de réception lorsque la première commande est reçue
  if(rx_index==0){
    for(int i=0; i<RX_BUFFER_SIZE; i++){
      RX_buffer[i]=0;
    }
  }
  
  // Analyse du caractère reçu
  switch(RX_received_char){
    case '#':
      rx_index = 0; // Réinitialiser l'indice du tampon de réception
      command_flag = 1; // Marquer le début d'une nouvelle commande
      break;
    case '%':
    case '\n':
    case '\r':
      // Vérifier si une commande a été détectée
      if(command_flag){
        RX_buffer[rx_index]=0; // Terminer la chaîne de caractères
        get_command(RX_buffer); // Traiter la commande complète
        command_flag=0; // Réinitialiser le drapeau de commande
        rx_index = 0; // Réinitialiser l'indice du tampon de réception
      }
      break;
    default:
      RX_buffer[rx_index++]=RX_received_char; // Stocker le caractère dans le tampon de réception
      break;
  }
}
