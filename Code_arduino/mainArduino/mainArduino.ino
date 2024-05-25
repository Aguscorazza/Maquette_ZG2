#include <util/atomic.h>

#define F_CPU 16000000

// UART DEFINITIONS
#define UART_BAUDRATE     38400
#define TX_BUFFER_SIZE    20
#define RX_BUFFER_SIZE    20


// ARDUINO PIN DEFINITIONS
#define LED_PIN           13
#define DIR_MOTOR_PIN     12
#define BRAKE_MOTOR_PIN   9
#define PWM_MOTOR_PIN     3
#define ENCODER_A_PIN     2


// CONTROL DEFINITIONS
#define MAX_TENSION       12    // V
#define ENCODER_CPR       64    // Encoder Counts per Revolution (CPR)
#define MOTOR_GEAR_RATIO  50
#define MAX_I_ERROR       100
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
float deltaT = 0.0;
float eintegral = 0;

typedef struct{
  int mode=0;                       // 0: Oscillation Libre - 1: Oscillation forcee
  float commanded_motor_speed=0.0;  // Motor speed (consigne) [rpm]
  
  float Kp = 20.0;
  float Ki = 10.0;
  
  unsigned int Te = 50;                      // Période d'échantillonnage [ms]
  
  float measured_motor_speed=0.0;           // Motor Speed [rpm]
  float measured_motor_current=0.0;         // Motor current [A]
  int measured_position=0;                  // Position [m]
  
  bool simulation_running=false;
  uint16_t T1_prescaler = 0;
} Simulation;

char TX_buffer[TX_BUFFER_SIZE];
char RX_buffer[RX_BUFFER_SIZE];
char TEST_BUFFER[TX_BUFFER_SIZE];
volatile int rx_index=0, command_flag=0;

Simulation simulation;

void setMotor(int dir, int pwmVal){
  digitalWrite(BRAKE_MOTOR_PIN,LOW); 
  analogWrite(PWM_MOTOR_PIN,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(DIR_MOTOR_PIN,HIGH);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(DIR_MOTOR_PIN,LOW);
  }
  else{
    // Or dont turn
    digitalWrite(BRAKE_MOTOR_PIN,HIGH);   
  }
}
void setMotor(int pwmVal){
  analogWrite(PWM_MOTOR_PIN,pwmVal); // Motor speed
}

void TIMER1_init(bool enable, unsigned int T){
  // Configure TIMER 1
  // This timer will periodically send data to the PC via the UART
  // Reset Prescaler (Stop Timer)
  TCCR1B &= ~(7<<CS10);
  TCCR1A = 0;
  TCNT1 = 0;
  if(enable) {
    
    float period = (float) (T/1000.0);
    float max_period = (float) (pow(2,16)/F_CPU);
    unsigned int prescaler_T1 = 0;
    if(period <= max_period) prescaler_T1=1;
    else if(period <= 8*max_period) prescaler_T1=8;
    else if(period <= 64*max_period) prescaler_T1=64;
    else if(period <= 256*max_period) prescaler_T1=256;
    else if(period <= 1024*max_period) prescaler_T1=1028;
    else prescaler_T1=0; // Desactivate TIMER in case excessive period

    
    switch(prescaler_T1) {
      case 1: //prescaler x1
      simulation.T1_prescaler |= (1<<CS10);
      break;
      case 8: //prescaler x8
      simulation.T1_prescaler |= (2<<CS10);
      break;
      case 64: //prescaler x64
      simulation.T1_prescaler |= (3<<CS10);
      break;
      case 256: //prescaler x256
      simulation.T1_prescaler |= (4<<CS10);
      break;
      case 1028: //prescaler x1024
      simulation.T1_prescaler |= (5<<CS10);
      break;
      default:
      case 0: //Timer off
      simulation.T1_prescaler &= ~(7<<CS10);
      break;
    }
    
    OCR1A = (uint16_t)(((period*F_CPU)/prescaler_T1)-1);
  }
  TCCR1B |= (1 << WGM12);
  
  // Enable Timer Overflow Interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void timer_on(){
  // Prescaler x1024
  TCNT1 = 0;
  TCCR1B |= simulation.T1_prescaler;
}

void timer_off(){
  // Reset Prescaler (Stop Timer)
  TCCR1B &= ~(1<<CS10);
  TCCR1B &= ~(1<<CS11);
  TCCR1B &= ~(1<<CS12);
  TCNT1 = 0;
}

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
      digitalWrite(BRAKE_MOTOR_PIN, LOW);
      break;
      
    case 0: // Oscillation libre
    default:
      digitalWrite(BRAKE_MOTOR_PIN,HIGH);
      setMotor(0);
      break;
  }   
  // Start simulation
  simulation->simulation_running=true;
  timer_on();
}

void stop_simulation_command(const char *command, Simulation *simulation){
  simulation->simulation_running=false;
  timer_off();
  // Arrêter le moteur
  setMotor(0);
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

  TIMER1_init(true, simulation.Te);
  // Configuration des broches
  pinMode(LED_PIN, OUTPUT);                                          // Configure la broche 13 en sortie
  pinMode(DIR_MOTOR_PIN, OUTPUT);                                          // Broche Arduino réservée pour le sens de rotation du moteur A
  pinMode(BRAKE_MOTOR_PIN, OUTPUT);                                           // Broche Arduino réservée pour le freinage du moteur A
  pinMode(PWM_MOTOR_PIN, OUTPUT);                                           // Broche PWM pour le contrôle de la vitesse du moteur A
  pinMode(ENCODER_A_PIN, INPUT);                                            // Broche d'entrée pour l'encodeur avec résistance de pull-up activée
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING); // Associe la fonction d'interruption encoderISR à la broche d'encodeur, déclenchée sur front-montant

  // État initial des broches
  digitalWrite(DIR_MOTOR_PIN, HIGH); // Le moteur A tourne dans le sens normal
  digitalWrite(BRAKE_MOTOR_PIN, HIGH);  // Activation du frein moteur A
  analogWrite(PWM_MOTOR_PIN, 0);      // Pas de vitesse pour le moteur A (PWM)

  interrupts(); // Réactive les interruptions après la configuration initiale

  put_string("UART OK"); // Envoie un message indiquant que la configuration UART est terminée
}

void loop() {
  if(simulation.simulation_running==true){
    if((micros()-prevT)/1.0e3>=(simulation.Te-deltaT)){
      // Mesurer la distance de la masse
      simulation.measured_position = get_distance();
      
      // read the encoder counts in an atomic block
      // to avoid potential misreads
      int pos = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        pos = pos_i;
      }
      
      long currT = micros();
      deltaT = ((float) (currT-prevT))/1.0e6;
      float velocity1 = (pos - posPrev)/deltaT;
      posPrev = pos;
      prevT = currT;
      
      // Convert count/s to RPM
      float v1 = velocity1/800.0*60.0;
      // Low-pass filter (25 Hz cutoff)
      simulation.measured_motor_speed = (0.854*simulation.measured_motor_speed + 0.0728*v1 + 0.0728*v1Prev)*2.0*pi/60.0; // [rpm]-->[rad/s]
      v1Prev = v1;
      
      // Measure the motor current
      simulation.measured_motor_current = map(analogRead(A0), 0, 1024, 0, 5000)*2.0/(1000.0*3.3);
        
      if(simulation.mode==1){
        // Compute the control signal u
        float e = simulation.commanded_motor_speed-simulation.measured_motor_speed;
        eintegral = eintegral + e*deltaT;

        if(eintegral > MAX_I_ERROR){
          eintegral = MAX_I_ERROR;
        } else if(eintegral< -MAX_I_ERROR){
          eintegral = -MAX_I_ERROR;
        }
        
        float u = simulation.Kp*e + simulation.Ki*eintegral;
        // Set the motor speed and direction
        int dir = 1;
        if (u<0){
          dir = -1;
        }
        int pwr = (int) fabs(u);
        if(pwr > 255){
          pwr = 255;
        }
        setMotor(dir,pwr);
      }
    }
  }

}


void encoderISR(){ 
  pos_i++; // Incrément du compteur de l'encodeur
}
 
ISR(USART_RX_vect){
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

ISR(TIMER1_COMPA_vect) {
  // Envoyer les données périodiquement
  sprintf(TX_buffer, "#P%d-V%d-I%d%", simulation.measured_position, simulation.measured_motor_speed, simulation.measured_motor_current);
  put_string(TX_buffer);
}
