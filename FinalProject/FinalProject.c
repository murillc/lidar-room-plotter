#include <stdint.h>
#include <math.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

// Carlos Murillo
// 400197550
// murillc
// Final Project C code

//uint16_t	dev = 0x29;
uint16_t	dev=0x52;

int status=0;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 0 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//Init Methods =======================================================================================
void PortL_Init(void){	//L1 Displacement Status
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;		              // activate the clock for Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	        // allow time for clock to stabilize
  GPIO_PORTL_DEN_R = 0b00000010;                         		// Enabled both as digital outputs
	GPIO_PORTL_DIR_R = 0b00000010;
	return;
}

void PortN_Init(void){ //N0 and N1 used for on board LEDs
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 //activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
	GPIO_PORTN_DIR_R=0b00111111;
	GPIO_PORTN_DEN_R=0b00111111;
	return;
}

void PortF_Init(void){ //initialize port F0 and F4 for on board LEDs
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 //activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};
	GPIO_PORTF_DIR_R=0b00010001;
	GPIO_PORTF_DEN_R=0b00010001;
	return;
}

void PortE_Init(void){	//E0 for button row
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;		              // activate the clock for Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};	        // allow time for clock to stabilize
  GPIO_PORTE_DEN_R = 0b00000001;                         		// Enabled both as digital outputs
	GPIO_PORTE_DIR_R = 0b00000001;
	GPIO_PORTE_DATA_R = 0b00000000;
	return;
}

void PortM_Init(void){ //M0 for button column
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 //activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        //allow time for clock to stabilize 
	GPIO_PORTM_DIR_R = 0b00000000;       								    // make PM0 an input, PM0 is reading if the button is pressed or not 
  GPIO_PORTM_DEN_R = 0b00000001;
	return;
}

void PortK_Init(void){ //K0-K3 used for stepper
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9;                 //activate the clock for Port k
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R9) == 0){};
	GPIO_PORTK_DIR_R=0xFF;
	GPIO_PORTK_DEN_R=0xFF;
	return;
}

//Stepper Methods ====================================================================================
	uint8_t stepperOn = 0; //keeps track of state of stepper
	uint8_t stepperStates[4] = {0b1100, 0b0110, 0b0011, 0b1001}; //stores states of stepper
	uint8_t currentState = 0;  //keeps track of current state of magnets
	uint8_t angle = 0;

void incrementCurrentState(uint8_t* currentState){ //increments state, looping back to 0 if 4 is reached
	*currentState = *currentState+1;
	if (*currentState == 4){
		*currentState = 0;
	}
}

void decrementCurrentState(uint8_t* currentState){ //increments state, looping back to 0 if 4 is reached
	if (*currentState == 0){
		*currentState = 3; //if its 0, make 3.  Have to do this way because unsigned int
		return;
	}
		*currentState = *currentState-1;
}

void rotateStepper(int dir, int speed){
	GPIO_PORTK_DATA_R = stepperStates[currentState];
	
	if (dir == 1) // If the dir is 1 go clockwise
		incrementCurrentState(&currentState);
	else  // If the dir is -1 (or anything else) go counter-clockwise
		decrementCurrentState(&currentState);	
	
	SysTick_Wait10ms(speed);
}

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	// port init
	PortE_Init();
	PortF_Init();
	PortK_Init();
	PortM_Init();
	PortN_Init();
	PortL_Init();
	
	// Wait until PC is ready
	int input = UART_InChar();
	while (input != 0x31) {
		input = UART_InChar(); // while loop needed or else if python is run first then microcontroller won't run
	}
	
	// Preliminary status prints
	UART_printf("Program Begins\r\n");
	sprintf(printf_buffer,"2DX4 Final Project\r\n");
	UART_printf(printf_buffer);
	sprintf(printf_buffer,"Carlos Murillo\r\n");
	UART_printf(printf_buffer);
	sprintf(printf_buffer,"400197550\r\n");
	UART_printf(printf_buffer);


	/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
  myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n");
 	UART_printf("One moment...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	Status_Check("StartRanging", status);

	// i = 0; Formerly used for debugging
	
	// Tell PC the ToF chip is ready and can begin collecting data
	// This also prevents the button from being pressed before it's ready
	UART_OutChar(0x1B);
	
	while(1){
		// If the button is pushed...
		if((GPIO_PORTM_DATA_R&0b00000001)==0){
			stepperOn = 1;
		}
		else {
			stepperOn = 0;
		}
		
		// If the button is pushed, run
		if (stepperOn) {
			// Turn off displacement LED
			GPIO_PORTL_DATA_R = 0b00000000;
			
			// Full-stepping
			for (int steps = 0; steps < 2048; steps++) {
				// Rotate counterclockwise at 10 ms delay per step
				rotateStepper(-1, 1);
				
				// If button is pressed again, pause measurements
				if (steps > 10) {
					if((GPIO_PORTM_DATA_R&0b00000001)==0){
						UART_OutChar(0x06); // Tell PC to pause measurements
						
						SysTick_Wait10ms(100); // Wait 1 s to avoid accidental inputs
						
						while ((GPIO_PORTM_DATA_R&0b00000001)!=0) {
							// Measurements are paused until button is pressed again
						}
						
						UART_OutChar(0x1B); // Tell PC to resume measurements
						
						SysTick_Wait10ms(50); // Wait 0.5s to avoid accidental inputs
					}
				}
				
				// 64 measurements. 2048/64 = every 32 steps, take a measurement
				if (steps % 32 == 0) {
					// Turn Distance LED on
					GPIO_PORTN_DATA_R = 0b000000010;
					
					// Wait for data ready
					while (dataReady == 0){
							status = VL53L1X_CheckForDataReady(dev, &dataReady);
							FlashLED3(1);
							VL53L1_WaitMs(dev, 5);
					}
					
					dataReady = 0;
					
					// Get distance
					status = VL53L1X_GetDistance(dev, &Distance);

					status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
					
					// Formerly used for debugging
					// debugArray[i++] = Distance
					
					// Send distance to PC through a string
					sprintf(printf_buffer, "%u\n", Distance);
					UART_printf(printf_buffer);
					
				}
				
			}
			
			// Turn Distance LED off
			GPIO_PORTN_DATA_R = 0b00000000;
			
		} else {
			// Turn Displacement LED on
			GPIO_PORTL_DATA_R = 0b00000010;
		}
	}
	


}


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        // 8) configure for 100 kbps clock
        
  // 20*(TPR+1)*20ns = 10us, with TPR=24
    // TED 100 KHz
    //     CLK_PRD = 8.3ns
    //    TIMER_PRD = 1
    //    SCL_LP = 6
    //    SCL_HP = 4
    //    10us = 2 * (1 + TIMER_PRD) * (SCL_LP + SCL_HP) * CLK_PRD
    //    10us = 2 * (1+TIMER+PRD) * 10 * 8.3ns
    //  TIMER_PRD = 59 (0x3B)
    //
    // TIMER_PRD is a 6-bit value.  This 0-127
    //    @0: 2 * (1+ 0) * 10 * 8.3ns --> .1667us or 6.0MHz
    //  @127: 2 * (1+ 127) * 10 * 8.3ns --> 47kHz
    
    
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

