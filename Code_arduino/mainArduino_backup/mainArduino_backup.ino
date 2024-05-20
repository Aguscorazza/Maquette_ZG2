#define UART_BAUDRATE     38400
#define TX_BUFFER_SIZE    20
#define RX_BUFFER_SIZE    20

#define FREQ_TIM1         20      //Hz
#define FREQ_PWM          25000     // Hz

#define MAX_TENSION       12    // V
#define ENCODER_CPR       64    // Encoder Counts per Revolution (CPR)
#define MOTOR_GEAR_RATIO  50
#define MAX_I_ERROR       1000
#define pi                3.1416

#define Kp 0.001
#define Ki 0.0001
#define Kt 38.4615            // Motor Torque Constant
#define Ke (Kt)
#define Ra 2.2
#define La 0.0023
#define R 72.2566             // 2*PI*fc*La // fc = 5000hz
#define bm 0.36728            // No-load current * Kt / No-load speed 


typedef struct{
  int duration;                 // Simulation duration
  int mode;                     // 0: Oscillation Libre - 1: Oscillation forcee
  double commanded_motor_speed;  // Motor speed (consigne) [rad/s]
  
  float i_error;
  double control_current, control_torque, control_tension;
  uint8_t control_rpm;
  uint16_t output_dutycycle;
  
  int encoder_count;            // Number of counts of the encoder signal (used to calcule motor speed)
  double measured_motor_speed;   // Motor speed (calculation) [rad/s]
  int measured_motor_speed_rpm;
  double measured_motor_current;  // Motor current [A]
  int measured_position;        // Position (calculation)
} Simulation;

char TX_buffer[TX_BUFFER_SIZE];
char RX_buffer[RX_BUFFER_SIZE];
char TEST_BUFFER[TX_BUFFER_SIZE];
volatile int rx_index=0, command_flag=0;

Simulation simulation;
int start_simulation_flag = 0; // Set this variable to start the simulation
int TIM1_OVF_counter = 0;

volatile bool newData = false;
char receivedData;

void PWM_init() {
  // Configure TIMER2 as PWM (Motor Input PWM)
  // Pin configuration
  DDRD |= (1<<DDD3);    // OUTPUT - Arduino pin 3 - Motor PWM (MOTOR A) (OC2B)
  
  // Output Mode (OC2B) Configuration (FAST PWM - Clear OC2B on compare match, set OC2B at BOTTOM)
  TCCR2A |= (1<<COM2B1);
  TCCR2A &= ~(1<<COM2B0);
  
  // Timer Operation Mode (FastPWM - TOP = OCR2A)
  TCCR2A |= (1<<WGM20);
  TCCR2A |= (1<<WGM21); 
  TCCR2B |= (1<<WGM22);
  
  TCNT2 = 0;
  // Configure Prescaler x8 (base freq = 16MHz / 8 = 2MHz)
  TCCR2B &= ~(1<<CS22);
  TCCR2B |= (1<<CS21);
  TCCR2B &= ~(1<<CS20);
  
  // Configure TOP = OCR2A
  OCR2A = (F_CPU / (8 * FREQ_PWM)) - 1; // With Prescaler x8
}

void PWM_set_duty(int duty) {
  if(duty>100){
    duty = 100;
  }
  if(duty>=0){
    analogWrite(3, duty);
  }
}

void PWM_on() {
  TCNT2 = 0;
  // Configure Prescaler x8 (base freq = 16MHz / 8 = 2MHz)
  TCCR2B &= ~(1<<CS22);
  TCCR2B |= (1<<CS21);
  TCCR2B &= ~(1<<CS20);
  
}

void PWM_off() {
  // Turn off TIMER
  TCCR2B &= ~(1<<CS22);
  TCCR2B &= ~(1<<CS21);
  TCCR2B &= ~(1<<CS20);
  TCNT2 = 0;
}

void PIController(float setpoint){
  float speed_error = setpoint-simulation.measured_motor_speed;
  simulation.i_error += speed_error; // Integral error
  
  // Anti-windup mechanism to limit the integral term
  if (simulation.i_error > MAX_I_ERROR) {
      simulation.i_error = MAX_I_ERROR;
  } else if (simulation.i_error < -MAX_I_ERROR) {
      simulation.i_error = -MAX_I_ERROR;
  }

  simulation.control_torque = Kp * speed_error + Ki * simulation.i_error;
  simulation.control_current = (simulation.control_torque + bm * simulation.measured_motor_speed)/Kt;
  simulation.control_tension = R*(simulation.control_current - simulation.measured_motor_current) + Ke * simulation.measured_motor_speed + Ra * simulation.measured_motor_current;

  simulation.output_dutycycle = (simulation.control_tension * 100) / (MAX_TENSION);
  PWM_set_duty(simulation.output_dutycycle);
}

void TIMER_init(){
  // Configure TIMER 1
  // This timer will periodically send data to the PC via the UART
  // Reset Prescaler (Stop Timer)
  TCCR1B &= ~(1<<CS10);
  TCCR1B &= ~(1<<CS11);
  TCCR1B &= ~(1<<CS12);
  
  // Operation mode (FastPWM - TOP: OCR1A)
  TCCR1A |= (1<<WGM10);
  TCCR1A |= (1<<WGM11);
  TCCR1B |= (1<<WGM12);
  TCCR1B |= (1<<WGM13);

  // Set TOP Value (Check FREQ_TIM_1 in config.h)
  OCR1A = (F_CPU/(1024*FREQ_TIM1)) - 1; //Using a x1024 Prescaler
  
  // Enable Timer Overflow Interrupt
  TIFR1 |= (1<<TOV1); // Clear flag
  TIMSK1 |= (1<<TOIE1);
}

void timer_on(){
  // Prescaler x1024
  TCNT1 = 0;
  TCCR1B |= (1<<CS10);
  TCCR1B &= ~(1<<CS11);
  TCCR1B |= (1<<CS12);
}

void timer_off(){
  // Reset Prescaler (Stop Timer)
  TCCR1B &= ~(1<<CS10);
  TCCR1B &= ~(1<<CS11);
  TCCR1B &= ~(1<<CS12);
  TCNT1 = 0;
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
    default:
      break;
  }
}

void start_simulation_command(const char *command, Simulation *simulation){
  sscanf(command, "S_%d_%d_%d", &(simulation->duration), &(simulation->mode), &(simulation->commanded_motor_speed));
  simulation->commanded_motor_speed = 2.0*pi*simulation->commanded_motor_speed/60.0; // rad/s
  // Start timer during the specified duration (simulation.duration)

  switch(simulation->mode){
    case 1: //Oscillation forcée
      
      digitalWrite(9, LOW);
      PWM_on();
      break;
      
    case 0: // Oscillation libre
    default:
      digitalWrite(9,HIGH);
      PWM_set_duty(0);
      PWM_off();
      break;
  }   
  
  digitalWrite(13, HIGH);
  TIM1_OVF_counter = 0;
  timer_on();
}


int get_distance(){
  int t = pulseIn(8, HIGH);//branché sur le pin 5
  if (t != 0 && t <= 1850)
    {
      return (t - 1000) * 2 ; // Convertir la largeur d'impulsion en microsecondes en distance en millimètres 
    }
}

void setup() {
  noInterrupts();                                               // Désactive les interruptions pendant la configuration initiale
  UART_init(UART_BAUDRATE, 1, 0);                               // Initialise la communication UART avec le taux de bauds spécifié
  TIMER_init();                                                 // Initialise les timers
  PWM_init();
  // Configuration des broches
  pinMode(13, OUTPUT); // Configure la broche 13 en sortie
  pinMode(12, OUTPUT);                                          // Broche Arduino réservée pour le sens de rotation du moteur A
  pinMode(9, OUTPUT);                                           // Broche Arduino réservée pour le freinage du moteur A
  pinMode(3, OUTPUT);                                           // Broche PWM pour le contrôle de la vitesse du moteur A
  pinMode(2, INPUT);                                            // Broche d'entrée pour l'encodeur avec résistance de pull-up activée
  attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING); // Associe la fonction d'interruption encoderISR à la broche d'encodeur, déclenchée sur front-montant

  // État initial des broches
  digitalWrite(12, HIGH); // Le moteur A tourne dans le sens normal
  digitalWrite(9, LOW);  // Activation du frein moteur A
  analogWrite(3, 100);      // Pas de vitesse pour le moteur A (PWM)
  
  interrupts(); // Réactive les interruptions après la configuration initiale

  put_string("UART OK"); // Envoie un message indiquant que la configuration UART est terminée
}

void loop() {
}

/**
 * Fonction d'interruption pour la gestion des compteurs d'encodeurs.
 * 
 * Cette fonction est appelée à chaque fois qu'une interruption est générée par l'encodeur.
 * Son rôle est d'incrémenter le compteur d'encodeurs pour suivre le nombre d'impulsions reçues.
 */
void encoderISR(){ 
  simulation.encoder_count++; // Incrément du compteur d'encodeurs
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


/**
 * Routine de service d'interruption (ISR) déclenchée par le dépassement de décompte du TIMER1.
 * 
 * Cette ISR est activée lorsque le TIMER1 dépasse sa valeur de décompte maximale. Elle est utilisée
 * pour effectuer plusieurs tâches périodiques, y compris le calcul de la vitesse du moteur, la mesure
 * de la distance parcourue, l'envoi de données via la communication série, et l'arrêt des timers
 * lorsque la simulation est terminée.
 */
ISR(TIMER1_OVF_vect){ // Freq = FREQ_TIM_1
  // Basculer la broche 13
  //digitalWrite(13, !digitalRead(13));
  

  
  if(simulation.mode==1){
    // Calculer la vitesse du moteur
    simulation.measured_motor_speed_rpm = (simulation.encoder_count * FREQ_TIM1 * 60)/(ENCODER_CPR*MOTOR_GEAR_RATIO); // RPM
    simulation.measured_motor_speed = simulation.measured_motor_speed_rpm * 2.0 * pi / 60.0; 
    simulation.encoder_count = 0;
  
    // Measure the motor current
    simulation.measured_motor_current = map(analogRead(A0), 0, 1024, 0, 5000)*2.0/(1000.0*3.3);
  
    PIController(simulation.commanded_motor_speed);
  }
  // Mesurer la distance parcourue
  simulation.measured_position = get_distance();

  // Envoyer les données
  sprintf(TX_buffer, "#P%d-V%d-I%d%", simulation.measured_position, simulation.measured_motor_speed_rpm, simulation.measured_motor_current);
  put_string(TX_buffer);

  // Désactiver les timers lorsque la simulation est terminée
  if(TIM1_OVF_counter/FREQ_TIM1>=simulation.duration){
    timer_off();
    // Arrêter le moteur
    PWM_set_duty(0);
    PWM_off();
    TIM1_OVF_counter = 0;
    digitalWrite(13, LOW);
  } else{
    TIM1_OVF_counter++;
  }
}
