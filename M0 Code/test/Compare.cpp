// Micros2 function
volatile unsigned long timer2Counter;
const byte LED1 = 12;


void setup() {
  Serial.begin(115200);                            // Set up the serial port for test purposes
 
  // Set up the generic clock (GCLK4) used to clock timers -----------------------------------------------------
 /* We are using clock 0 that is already setup.
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);             // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                      GCLK_GENCTRL_GENEN |         // Enable GCLK4
                      GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                      GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization
 */ 
  // Feed GCLK0 to TCC2 (and TC3)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK0 to TCC2 (and TC3)
                      GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0
                      GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  TC3->COUNT8.PER.reg = 0xFF;                      // Set period register to 255
  while (TC3->COUNT8.STATUS.bit.SYNCBUSY);         // Wait for synchronization

  TC3->COUNT8.INTENSET.reg = /*TC_INTENSET_MC1 | TC_INTENSET_MC0 |*/ TC_INTENSET_OVF; // Enable TC3 interrupts
 
  //NVIC_DisableIRQ(TC3_IRQn);
  //NVIC_ClearPendingIRQ(TC3_IRQn);
  NVIC_SetPriority(TC3_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC3 to 0 (highest)
  NVIC_EnableIRQ(TC3_IRQn);         // Connect TC3 to Nested Vector Interrupt Controller (NVIC)

  // Set the TC3 timer to tick at 2MHz, or in other words a period of 0.5us - timer overflows every 128us
  // timer counts up to (up to 255 in 128us)
  TC3->COUNT8.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8 |      // Set prescaler to 8, 16MHz/8 = 2MHz
                           TC_CTRLA_PRESCSYNC_PRESC |     // Set the reset/reload to trigger on prescaler clock
                           TC_CTRLA_MODE_COUNT8;          // Set the counter to 8-bit mode
                           
  TC3->COUNT8.CTRLA.bit.ENABLE = 1;               // Enable TC3
  while (TC3->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  TC3->COUNT8.READREQ.reg = TC_READREQ_RCONT |            // Enable a continuous read request
                            TC_READREQ_ADDR(0x10);        // Offset of the 8 bit COUNT register
  while (TC3->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for (read) synchronization
}

void loop() {
  Serial.println(micros2());                      // Testing the micros2() function
  delay(1000);                                    // Wait 1 second
}

// Micros2 is used to measure the receiver pulsewidths down to 1us accuracy
uint32_t micros2()
{
  uint32_t m;
  uint8_t t;
     
  noInterrupts();                                 // Disable interrupts
  m = timer2Counter;                              // Get the number of overflows
  t = TC3->COUNT8.COUNT.reg;                      // Get the current TC3 count value

  if (TC3->COUNT8.INTFLAG.bit.OVF && (t < 255))   // Check if the timer has just overflowed (and we've missed it)
  {
    m++;                                          // Then in this case increment the overflow counter
  } 
  interrupts();                                   // Enable interrupts
  return ((m << 8) + t) / 2;                      // Return the number of microseconds that have occured since the timer started
}

// This ISR is called every 128us
void TC3_Handler()           // ISR TC3 overflow callback function
{
  if (TC3->COUNT8.INTFLAG.bit.OVF)
  {
    timer2Counter++;           // Increment the overflow counter
  }
  TC3->COUNT8.INTFLAG.reg = TC_INTFLAG_OVF;   // Rest the overflow interrupt flag
}