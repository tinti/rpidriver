const int red = RED_LED;
const int green = GREEN_LED;
const int irq = P1_4;
const int button = PUSH2;
const int pin = P1_5;

const int red_control = P2_0;
const int green_control = P2_1;
const int pin_control = P2_2;

int red_value = 0;
int green_value = 0;
int pin_value = 0;
int button_value = 0;

int last_button_value = 0;

void setup() {                
  // LED
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  
  // IRQ
  pinMode(irq, OUTPUT);
  
  // PIN
  pinMode(pin, OUTPUT);
  
  // BUTTON
  pinMode(button, INPUT_PULLUP);
  
  // CONTROL
  pinMode(red_control, INPUT);
  pinMode(green_control, INPUT);
  pinMode(pin_control, INPUT);
  
  // set default values
  digitalWrite(red, LOW);
  digitalWrite(green, LOW);
  digitalWrite(irq, LOW);
  digitalWrite(pin, LOW);
  
  red_value = LOW;
  green_value = LOW;
  pin_value = LOW;
  button_value = HIGH;
  last_button_value = HIGH;
}

void loop() {
  // read data
  red_value = digitalRead(red_control); 
  green_value = digitalRead(green_control);
  pin_value = digitalRead(pin_control);
  button_value = digitalRead(button);
  
  // report value
  if ((button_value == LOW) && (last_button_value == HIGH)) {
    // change output value
    digitalWrite(pin, pin_value);
    
    // issue an irq
    digitalWrite(irq, HIGH);
    delay(100);
    digitalWrite(irq, LOW);
  }
  
  // set led values
  digitalWrite(red, red_value);
  digitalWrite(green, green_value);

  // update last button value 
  last_button_value = button_value;
  
  // delay
  delay(10);
}
