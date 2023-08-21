#define BUFFER_SIZE  256//Buffer size for each of the 3 peaks 
#define TIME_SIZE 6// There is going to be 6 start time and stop time for sampling. 2 for each peak
#define LOW_THRESHOLD 300// Minimum value of the incoming peak that stops the sampling 
#define HIGH_THRESHOLD 380//Minimum value of the incoming peak that triggers the sampling 
/*Change these threshold values to compensate the noise*/
#define THRESHOLD_GAP (HIGH_THRESHOLD-LOW_THRESHOLD)
#define CAVITY_REFERENCE 100000//t_M that sets the lockpoint for the cavity
#define CAVITY_DACC_OFFSET 2048
#define LASER_DACC_OFFSET 2048
#define HALF_RANGE 2047

union Float_convert
{
   uint8_t byte[4];
   float real;
};

union Int_convert
{
   uint8_t byte[2];
   uint16_t real;
};

float setpoint_change_factor = -341.2;//(beta^-1)*10^6. You can calcuate this using the cavity properties or calibrate through atomic constant measurements like A_{HFS}
//float LASER_REFERENCE_MOT = 631302;//This number is for Yb174
//float LASER_REFERENCE_MOT = 427250;//This number is for Yb174
float LASER_REFERENCE_MOT = 671173;//10^6*r that sets the slave laser setpoint during the 2D MOT stage. This number is for Yb171
volatile float LASER_REFERENCE = LASER_REFERENCE_MOT; //10^6*r that sets the slave laser setpoint

//Define variables that allow for scanning the frequency
volatile uint16_t steps = 1  ;         // number of steps
volatile uint16_t repeats = 1000;       // number of repeats

volatile float pp  = 171.7;  // Bias=-3V
volatile float pphigh  =510-5.8+3.5-2+2.2;  //Bias=-9V
volatile float mphigh  =-505.12;  //Bias=-9V

//volatile float mp  = 456;  // Bias=-3V 


//volatile float start_frequency  =  pphigh;  // -178.8;  //(units of MHz, -5 is optimal)
//volatile float stop_frequency   = pphigh; //-178.8; // 168.2; //(units of MHz)
volatile float start_frequency  =0;//200;//-30;  // zero bias
volatile float stop_frequency  = 0;//500;//=30;// 60 ;  // zero bias
volatile uint16_t step_counter =0;//keeps the current step number
volatile uint16_t repeat_counter = 0;//keeps the current repeat number
float frequency_jump = (stop_frequency - start_frequency) / steps; //Units of MHz
boolean jump = LOW;//Determines if the Setpoint jump should be invoked or not
volatile float frequency_changed = 0;


//Define Buffers and variables for storing acquired and processed peak data
uint16_t buf_1[BUFFER_SIZE];//Buffer for M peak
int16_t Length[3];//Keeps track of how many points were acquired for each peak
int16_t derv_buf_1[BUFFER_SIZE];//Buffer for the derivative of M peak
uint16_t buf_2[BUFFER_SIZE];//Buffer for S peak
int16_t derv_buf_2[BUFFER_SIZE];//Buffer for the derivative of S peak
uint16_t buf_3[BUFFER_SIZE];//Buffer for M' peak
int16_t derv_buf_3[BUFFER_SIZE];//Buffer for the derivative of M' peak
int32_t times[TIME_SIZE];//Keeps track of the start and stop time of the DMA acquisition for each peak
int32_t peak_times[3];//The time wrt to the Global digital trigger of the zero-crossing of each peak derivative
boolean flag;//If the program control has entered peakfinder subroutines. HIGH-yes, LOW-no 
volatile int counter;//Enumerates the acquired peak: M(0),S(1),M'(1)

//Define Servo Loop Variables
volatile float cavity_K_i = 0;
volatile float cavity_K_p = 0;
volatile float laser_K_i = 0;
volatile float laser_K_p = 0;
int32_t cavity_error_signal_current;
int32_t cavity_error_signal_prev;
float laser_error_signal_current;
float laser_error_signal_prev;
int32_t delta_cavity = 0;
float delta_laser = 0;
volatile float average_laser_error_signal = 0;
volatile float sum_laser_error_signal = 0;
int32_t cavity_accumulated_error_signal = 0;
int32_t cavity_control_signal = 0;
int32_t laser_control_signal = 0;
volatile uint16_t average = 1;
boolean bump = LOW;//I use the bump to indicate if the cavity is locked(HIGH) or not(LOW).
boolean is_broken = 0;
int32_t laser_control_signal_recorder = 0;
int32_t cavity_control_signal_recorder = 0;
int16_t flash_counter=0;
boolean flashing_status=0;
int16_t broken_counter=0;
int16_t broken_counter_cavity=0;
const uint16_t stringBufferSize = 11;
uint8_t stringBuffer[stringBufferSize];
boolean i2c_debug = 0;
boolean I2C_str_status = 0; // 0 to be start; 1 to be stop
boolean I2C_status = 0;
int32_t stringCounter = 0;
Float_convert f1;
Float_convert f2;
Int_convert in1;
// int test_c =0;

//Define variable for acquiring the Start Time of the SysTick/Global Timer
volatile uint32_t start_time = 0;

void setup()
{

  Serial.begin(57600);
  adc_setup();
  global_digital_interrupt_setup();
  dacc_setup();
  setpoint_change_interrupt_setup();
  I2C_init();
  pinMode(50,OUTPUT);
  delay(2000);
}
void loop()
{
      if (I2C_str_status && i2c_debug){
    printer();
    I2C_str_status =0;
  }

 if (counter == 0 && times[1])  //True when the M peak has been acquired
  {
    peakfinder_1();// Subroutine to process the M peak
  }


  if (counter == 1 && times[3])  //True when the S peak has been acquired
  {
    peakfinder_2();// Subroutine to process the S peak
  }

  if (counter == 2 && times[5])  //True when the M' peak has been acquired
  {
    peakfinder_1();// Subroutine to process the M' peak
  }

}
void setpoint_change_interrupt_setup()
{
  //Enable PB14
  PIOB->PIO_PER = PIO_PB14;
  //Set PB14 as input
  PIOB->PIO_ODR = PIO_PB14;
  //Disable pull-up on both pins
  PIOB->PIO_PUDR = PIO_PB14;
  pmc_enable_periph_clk(ID_PIOB);
  NVIC_DisableIRQ(PIOB_IRQn);
  NVIC_ClearPendingIRQ(PIOB_IRQn);
  NVIC_SetPriority(PIOB_IRQn, -2-1);
  NVIC_EnableIRQ(PIOB_IRQn);


  PIOB->PIO_AIMER = PIO_PB14;
  //Level Select Register
  PIOB->PIO_ESR = PIO_PB14;
  //Rising Edge/High Level Select Register
  PIOB->PIO_REHLSR = PIO_PB14;
  //Finally enable interrupts 
  PIOB->PIO_IER = PIO_PB14;
}
void adc_setup()
{


  //ADC Configuration
  analogReadResolution(12);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  ADC->ADC_CHER = ADC_CHER_CH6; 
  ADC->ADC_MR |= 0x80;
  adc_set_comparison_channel(ADC, ADC_CHANNEL_6);
  adc_set_comparison_mode(ADC, ADC_EMR_CMPMODE_HIGH);
  adc_set_comparison_window(ADC, LOW_THRESHOLD, HIGH_THRESHOLD);
  adc_enable_interrupt(ADC, ADC_IER_COMPE);
  adc_start(ADC);
  //ADC Interrupt NVIC Enable
  pmc_enable_periph_clk(ID_ADC);
  NVIC_DisableIRQ(ADC_IRQn);
  NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn, 0-4);
  NVIC_EnableIRQ(ADC_IRQn);
  analogWriteResolution(12);
}

void global_digital_interrupt_setup()
{ //Enable PC1
  PIOC->PIO_PER = PIO_PC1;
  //Set PC1 as input
  PIOC->PIO_ODR = PIO_PC1;
  //Disable pull-up on both pins
  PIOC->PIO_PUDR = PIO_PC1;
  pmc_enable_periph_clk(ID_PIOC);
  NVIC_DisableIRQ(PIOC_IRQn);
  NVIC_ClearPendingIRQ(PIOC_IRQn);
  NVIC_SetPriority(PIOC_IRQn, -1-1);
  NVIC_EnableIRQ(PIOC_IRQn);
 
  //Additional Interrupt Modes Enable Register
  PIOC->PIO_AIMER = PIO_PC1;
  //Edge Select Register
  PIOC->PIO_ESR = PIO_PC1;
  //Falling Edge/Low Level Select Register
  PIOC->PIO_REHLSR = PIO_PC1;
  //Finally enable interrupts
  PIOC->PIO_IER = PIO_PC1;



  PIOC->PIO_PER = PIO_PC4;
  //Set PC4 as input
  PIOC->PIO_ODR = PIO_PC4;
  //Disable pull-up on both pins
  PIOC->PIO_PUDR = PIO_PC4;
  
  //Additional Interrupt Modes Enable Register
  PIOC->PIO_AIMER = PIO_PC4;
  //Level Select Register
  PIOC->PIO_LSR = PIO_PC4;
  //Falling Edge/Low Level Select Register
  PIOC->PIO_FELLSR = PIO_PC4;
  //Finally enable interrupts
  PIOC->PIO_IER = PIO_PC4;
}

void dacc_setup()
{
  //DACC Configuration
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ; // start clocking DAC
  dacc_reset(DACC);
  dacc_set_transfer_mode(DACC, 0);//This allows for WORD transfer from the FIFO rather than HALF-WORD transfer
  dacc_enable_channel(DACC, 0);
  dacc_enable_channel(DACC, 1);
  //DACC Interrupt NVIC Enable
  NVIC_DisableIRQ(DACC_IRQn);
  NVIC_ClearPendingIRQ(DACC_IRQn);
  NVIC_SetPriority(DACC_IRQn, 0-4);
  NVIC_EnableIRQ(DACC_IRQn);
}

void ADC_Handler()
{ 
  if (((adc_get_status(ADC) & ADC_ISR_COMPE) == ADC_ISR_COMPE) & start_time != 0) 
  {
    ADC->ADC_EMR ^= 0x01;//toggles the comparision mode register
    NVIC_ClearPendingIRQ(ADC_IRQn);

    if (!(ADC->ADC_EMR & 0x01))
    {
      if (times[0] == 0)
      {
        times[0] = SysTick->VAL;//Acquires the start time of the DMA acquisition of M peak
        //Start the DMA acquisition for the M Peak
        REG_ADC_RPR = (uint32_t) buf_1;
        REG_ADC_RCR = BUFFER_SIZE;
        REG_ADC_PTCR = ADC_PTCR_RXTEN;
      }
      else if (times[2] == 0)
      {
        times[2] = SysTick->VAL;//Acquires the start time of the DMA acquisition of S peak
        //Start the DMA acquisition for the S Peak
        REG_ADC_RPR = (uint32_t) buf_2;
        REG_ADC_RCR = BUFFER_SIZE;
        REG_ADC_PTCR = ADC_PTCR_RXTEN;

      }
      else if (times[4] == 0)
      {
        times[4] = SysTick->VAL;//Acquires the start time of the DMA acquisition of M' peak
        //Start the DMA acquisition for the M' Peak
        REG_ADC_RPR = (uint32_t) buf_3;
        REG_ADC_RCR = BUFFER_SIZE;
        REG_ADC_PTCR = ADC_PTCR_RXTEN;

      }
    }
    else
    {
      if (times[1] == 0)
      {
        times[1] = SysTick->VAL;//Acquires the stop time of the DMA acquisition of M peak
        //Stop the DMA acquisition for the M Peak
        REG_ADC_PTCR = ADC_PTCR_RXTDIS;
        Length[0] = BUFFER_SIZE - REG_ADC_RCR;
      }
      else if (times[3] == 0)
      {
        times[3] = SysTick->VAL;//Acquires the stop time of the DMA acquisition of S peak
         //Stop the DMA acquisition for the S Peak
        REG_ADC_PTCR = ADC_PTCR_RXTDIS;
        Length[1] = BUFFER_SIZE - REG_ADC_RCR;
      }
      else if (times[5] == 0)
      {
        times[5] = SysTick->VAL;//Acquires the stop time of the DMA acquisition of M' peak
         //Stop the DMA acquisition for the M' Peak
        REG_ADC_PTCR = ADC_PTCR_RXTDIS;
        Length[2] = BUFFER_SIZE - REG_ADC_RCR;
       

      }

    }
  }

}

void PIOC_Handler()//Right now I am using the PIOC_handler from the internal functions to work out the interrupt. Will use a typedef function pointer. See WInterrupts.c and Stephen Prata
{
  
  if ((PIOC->PIO_ISR & PIO_PC1) == PIO_PC1)
  {
    //Setup up the Global Timer or the SysTick Timer
    SysTick->CTRL = 0;
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x5;
    while (SysTick->VAL != 0);
    start_time = SysTick->VAL;
    
    //Clear all the buffers
    memset(times, 0, sizeof(times));
    memset(buf_1, 0, sizeof(buf_1));
    memset(buf_2, 0, sizeof(buf_2));
    memset(buf_3, 0, sizeof(buf_3));
    memset(derv_buf_1, 0, sizeof(derv_buf_1));
    memset(derv_buf_2, 0, sizeof(derv_buf_2));
    memset(derv_buf_3, 0, sizeof(derv_buf_3));
    
    counter = 0;
  }
  if ((PIOC->PIO_ISR & PIO_PC4) == PIO_PC4)
  {
    cavity_K_p = 0.004;//Proportional Gain for the cavity PZT
    cavity_K_i = 0.001;//Integral Gain for the cavity PZT
    laser_K_i =0.01;//Proportional Gain for the laser PZT
    laser_K_p =0.001;//Integral Gain for the laser PZT
    bump = (HIGH && (!(bump)));
  }

  //These are the lock signals
  dacc_set_channel_selection(DACC, 1);
  dacc_write_conversion_data(DACC_INTERFACE, cavity_control_signal + CAVITY_DACC_OFFSET);
  dacc_set_channel_selection(DACC, 0);
  dacc_write_conversion_data(DACC_INTERFACE, laser_control_signal + LASER_DACC_OFFSET);

}
void PIOB_Handler()//This Handler processes TTL signals to perform jump in Slave laser frequency setpoint
{

  if ((PIOB->PIO_ISR & PIO_PB14 ) == PIO_PB14)
  {

    if (step_counter <= steps)
    {
      if (!jump)
      {
        LASER_REFERENCE +=  setpoint_change_factor * (start_frequency + step_counter * frequency_jump);
        jump = !jump;
        step_counter++;
      }
      else
      {
        LASER_REFERENCE = LASER_REFERENCE_MOT;
        jump = !jump;
      }

    }

    else
    {

      if (repeat_counter <= repeats)
      {
        LASER_REFERENCE = LASER_REFERENCE_MOT;
        step_counter = 0;
        repeat_counter++;
        jump=LOW;
      }
      
    }
    
  }


}
void peakfinder_1()//Subroutine dedicated to finding the M,M' peaks
{

  flag = HIGH;
  counter++;
  if (counter == 1)//Is this the M peak?
  {
    for (int i = 2; i <= (Length[0] - 3); i++)
    {
      derv_buf_1[i] = (((buf_1[i + 2] << 1) + buf_1[i + 1] - buf_1[i - 1] - (buf_1[i - 2] << 1)) ); //5-point Savitzky-Golay Filter.The denominator doesn't matter when determing zero-crossing.
      if (derv_buf_1[i] <= 0 && flag)
      {
        flag = LOW;
        peak_times[0] = start_time - (times[0] - i * (times[0] - times[1]) / Length[0]) + (float)(derv_buf_1[i] * (times[0] - times[1]) / (Length[0] * (derv_buf_1[i - 1] - derv_buf_1[i]))) ;
       //The line of code uses the method of linear interpolation near the zero crossing to calculate the peaktime

        
      }
    }
  NVIC_SetPendingIRQ(DACC_IRQn); //Software Triggered Interrupt that invokes the DACC Handler to perform Servo Loop calculations
  }
  if (counter == 3)//Is this the M' peak?
  {
    for (int i = 2; i <= (Length[2] - 3); i++)
    {
      derv_buf_3[i] = (((buf_3[i + 2] << 1) + buf_3[i + 1] - buf_3[i - 1] - (buf_3[i - 2] << 1)) );//5-point Savitzky-Golay Filter.The denominator doesn't matter when determing zero-crossing.
      if (derv_buf_3[i] <= 0 && flag)
      {
        flag = LOW;
        peak_times[2] = start_time - (times[4] - i * (times[4] - times[5]) / Length[2]) + (float)(derv_buf_3[i] * (times[4] - times[5]) / (Length[2] * (derv_buf_3[i - 1] - derv_buf_3[i]))) ;
        //The line of code uses the method of linear interpolation near the zero crossing to calculate the peaktime

        
      }
    }
  NVIC_SetPendingIRQ(DACC_IRQn); //Software Triggered Interrupt that invokes the DACC Handler to perform Servo Loop calculations
  }
 
}
void peakfinder_2()//Subroutine dedicated to finding the S peak
{
  counter++;
  flag = HIGH;
  //This is the S peak.
  for (int i = 2; i <= (Length[1] - 3); i++)
  {
    derv_buf_2[i] = (((buf_2[i + 2] << 1) + buf_2[i + 1] - buf_2[i - 1] - (buf_2[i - 2] << 1)) ); //5-point Savitzky-Golay Filter.The denominator doesn't matter when determing zero-crossing.
    //of 2.
     if (derv_buf_2[i] <= 0  && flag)
    {
      flag = LOW;
     
      peak_times[1] = start_time - (times[2] - i * (times[2] - times[3]) / Length[1]) + (float)(derv_buf_2[i] * (times[2] - times[3]) / ((derv_buf_2[i - 1] - derv_buf_2[i]) * Length[1])) ;
      //The line of code uses the method of linear interpolation near the zero crossing to calculate the peaktime

    }
  }


  NVIC_SetPendingIRQ(DACC_IRQn);//Software Triggered Interrupt that invokes the DACC Handler to perform Servo Loop calculations

}


void DACC_Handler()
{

 //Cavity and Slave laser frequency Locking Using the velocity algorithm

  if (!bump)//If the lock is not engaged, do not feedback on the cavity and laser frequency.
  { cavity_control_signal = 0;
    laser_control_signal = 0;
  }
  else
  {
    cavity_error_signal_current = CAVITY_REFERENCE - peak_times[0];

     delta_cavity = cavity_K_p * (cavity_error_signal_current - cavity_error_signal_prev) + cavity_K_i * (cavity_error_signal_current);
    if (((cavity_control_signal + delta_cavity) > HALF_RANGE) || ((cavity_control_signal + delta_cavity) < -(HALF_RANGE + 1)))
    {
      cavity_control_signal += 0;//If the delta_cavity is so large that it will put the control signal outside the rails, do not update the control signal
    }
    else
    {
      cavity_control_signal += delta_cavity;
    }

    
    laser_error_signal_current =  LASER_REFERENCE - 1000000 * (float)(peak_times[1] - peak_times[0]) / (peak_times[2] - peak_times[0]);

    delta_laser = -laser_K_p * (laser_error_signal_current - laser_error_signal_prev) - (laser_K_i * laser_error_signal_current ) ;



    if (((laser_control_signal + delta_laser) > HALF_RANGE) || ((laser_control_signal + delta_laser) < -(HALF_RANGE + 1)))
    {
      laser_control_signal += 0;//If the delta_cavity is so large that it will put the control signal outside the rails, do not update the control signal
     }
    else
    {
      laser_control_signal += delta_laser;
    }
  }

  cavity_error_signal_prev = cavity_error_signal_current;
  laser_error_signal_prev = laser_error_signal_current;

  // Enable this section if you want to get the average slave laser setpoint and then print the average_laser_error_signal
 /* if (average <= 1000)
  {
    sum_laser_error_signal += -laser_error_signal_current;
    average++;
  }
  else
  {
    average_laser_error_signal = sum_laser_error_signal / 1000;
    average = 1;
    sum_laser_error_signal = 0;
   
  }
  */
  if (laser_control_signal_recorder){
    if (laser_control_signal == laser_control_signal_recorder){
      broken_counter ++;
    }else{
        broken_counter = 0;
    }
  }
  if (cavity_control_signal_recorder){
    if (cavity_control_signal == cavity_control_signal_recorder){
      broken_counter_cavity ++;
    }else{
        broken_counter_cavity = 0;
    }
  }
  if (broken_counter >10 or broken_counter_cavity>200){
    is_broken = 1;
  }else{
    is_broken = 0;
  }
  laser_control_signal_recorder = laser_control_signal;
  cavity_control_signal_recorder = cavity_control_signal;
  if (bump && !is_broken)
  {
    if (abs(laser_control_signal_recorder)>1500){
        if (!flash_counter){
            digitalWrite(50,LOW);
            flashing_status = 0;
        }else{
            if (flash_counter > 50){
                flashing_status = !flashing_status;
                digitalWrite(50,flashing_status);
                flash_counter = 0;
            }
        }
        flash_counter ++;
    }else{
        digitalWrite(50,HIGH);
        flash_counter = 0;
    }
  }
  else{
    digitalWrite(50,LOW);
  }
    if (I2C_str_status && i2c_debug){
      printer();
    I2C_str_status =0;
  }
}

// void spi_interrupt_setup(){
//   SPI0->SPI_CR = SPI_CR_SPIDIS;
//   SPI0->SPI_CR = SPI_CR_SWRST;
//   SPI0->SPI_CR = SPI_CR_SWRST;
//   delay(10);

//   SPI0->SPI_MR = SPI_MR_MODFDIS             // Disable Mode Fault detection
//                  | SPI_MR_PCS(0b1110);

//   SPI0->SPI_CSR[0] |= SPI_CSR_CPOL          // Inactive state value of SPCK is logic level one
//                       | SPI_CSR_NCPHA       // Data is captured on the leading edge of SPCK and changed on the following edge
//                       | SPI_CSR_CSNAAT      // Chip select active after transfer
//                       | SPI_CSR_BITS_16_BIT // Bits per transfer
//                       | SPI_CSR_SCBR(100);   // slowest bit rate

  
//   SPI0->SPI_IER = SPI_IER_RDRF; // It is worth noting that the SPI is active-low on the CS lane
//                               // So the interrupt used here could be RDRF
//   NVIC_EnableIRQ(SPI0_IRQn);
//   SPI0->SPI_CR = SPI_CR_SPIEN;
// }

// void SPI0_Handler() {
//   stringCounter = 0;
//   stringBuffer[0] = '\0';
//   endOfLine = 0;
//   while (!endOfLine){
//     DataReceived = SPI0->SPI_RDR & SPI_RDR_RD_Msk;
//     if (DataReceived == '\n'){
//       endOfLine = 1;
//       stringBuffer[stringCounter] = '\0';
//     }else{
//       stringBuffer[stringCounter] = DataReceived;
//       stringCounter ++;
//     }
//   }
//   char * token = strtok(stringBuffer, ";");
//   start_frequency = atof(token);
//   token = strtok(NULL, ";");
//   stop_frequency = atof(token);
//   token = strtok(NULL, ";");
//   steps = atoi(token);
// }

void I2C_init(){
  PMC->PMC_PCER0 |= PMC_PCER0_PID22; 
  pinMode(71, INPUT_PULLUP);
  pinMode(70, INPUT_PULLUP);
  PIO_Configure(
					   g_APinDescription[PIN_WIRE1_SDA].pPort,
					   g_APinDescription[PIN_WIRE1_SDA].ulPinType,
					   g_APinDescription[PIN_WIRE1_SDA].ulPin,
					   g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
	PIO_Configure(
					   g_APinDescription[PIN_WIRE1_SCL].pPort,
					   g_APinDescription[PIN_WIRE1_SCL].ulPinType,
					   g_APinDescription[PIN_WIRE1_SCL].ulPin,
					   g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);
  NVIC_DisableIRQ(TWI0_IRQn);
  NVIC_ClearPendingIRQ(TWI0_IRQn);
  // NVIC_SetPriority(TWI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 3));
  NVIC_SetPriority(TWI0_IRQn, -5);
  NVIC_EnableIRQ(TWI0_IRQn);
  TWI0->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;	// Disable PDC channel
  TWI_ConfigureSlave(TWI0, 0x11);	// set to master mode
  TWI0->TWI_IDR = TWI0 -> TWI_IMR;			// disable all interrupts
  TWI0->TWI_CR = TWI_CR_MSDIS | TWI_CR_SVEN;
  TWI0->TWI_IER = TWI_IER_SVACC;
  TWI0->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
  // Serial.println("I2C Inited");
}

void TWI0_Handler() {
  if (!I2C_status){
    TWI0->TWI_RPR = (RwReg)stringBuffer;		
    TWI0->TWI_RCR = stringBufferSize;
    TWI0->TWI_RNPR = 0;
    TWI0->TWI_RNCR = 0;
    TWI0->TWI_PTCR = TWI_PTCR_RXTEN;
    TWI0->TWI_IDR = TWI0 -> TWI_IMR;
    TWI0->TWI_IER = TWI_IER_EOSACC;
    I2C_status = 1;
  }else{
    TWI0->TWI_PTCR = TWI_PTCR_RXTDIS;
    TWI0->TWI_IDR = TWI0 -> TWI_IMR;
    TWI0->TWI_IER = TWI_IER_SVACC;
    if (stringBuffer[10] == 17){
      f1.byte[0] = stringBuffer[0];
      f1.byte[1] = stringBuffer[1];
      f1.byte[2] = stringBuffer[2];
      f1.byte[3] = stringBuffer[3];
      f2.byte[0] = stringBuffer[4];
      f2.byte[1] = stringBuffer[5];
      f2.byte[2] = stringBuffer[6];
      f2.byte[3] = stringBuffer[7];
      in1.byte[0] = stringBuffer[8];
      in1.byte[1] = stringBuffer[9];
      start_frequency = f1.real;
      frequency_jump = f2.real;
      steps = in1.real;
    }
    I2C_str_status = 1;
    I2C_status = 0;
    // test_c ++;
    TWI0 -> TWI_SR & TWI_SR_EOSACC;
  }
}

void printer(){
    for (int i=0;i<stringBufferSize;i++){
      Serial.print(stringBuffer[i]);
      Serial.print(',');
    }
    Serial.println();
    Serial.print("Received: ");
    Serial.print(start_frequency);
    Serial.print(',');
    Serial.print(stop_frequency);
    Serial.print(',');
    Serial.println(steps);
    // Serial.println(test_c);
}
