/*
 * Pmod DHB1 C++ Example Application
 */

// xilinx headers
#include "xil_cache.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xintc.h"

// sensors
#include "PmodDHB1.h"
#include "PWM.h"
#include "MotorFeedback.h"
#include "PmodMAXSONAR.h"
#include <sys/_intsup.h>
#include <sys/_types.h>


// Pmod DHB1
#define PMOD_DHB1_CLOCK_FREQ_HZ XPAR_CPU_CORE_CLOCK_FREQ_HZ

#define M1_CHANNEL 1
#define M2_CHANNEL 2

#define PWM_PERIOD 0x00029000 // 2ms
#define PWM_DUTY 0x00014800   // 50% duty cycle
#define PWM_M1 0
#define PWM_M2 1

#define XPAR_PMOD_DHB1_0_GPIO_BASEADDR XPAR_PMOD_DHB1_0_BASEADDR
#define XPAR_PMOD_DHB1_0_PWM_BASEADDR 0x44A20000

XGpio DHB1_GPIO;
XGpio LS1_GPIO;
PmodDHB1 motor;
#define AXI_GPIO_0_BASE_ADDR 0x40000000
#define AXI_GPIO_1_BASE_ADDR 0x40010000 

#define PMOD_SONAR0_BASEADDR XPAR_PMOD_DUAL_MAXSONAR_0_BASEADDR
#define PMOD_LS1_BASEADDR 0x40020000

#define L_SENSOR 0x1
#define R_SENSOR 0x2
#define BOTH_SENSORS 0x3

#define RGB_LEDS_BASE_ADDR (AXI_GPIO_1_BASE_ADDR)
#define RGB_LEDS_REG (unsigned *)(RGB_LEDS_BASE_ADDR)

#define SWITCH_BTNS_BASE_ADDR (AXI_GPIO_1_BASE_ADDR + 8)
#define SWITCH_BTNS_REG (unsigned *) (SWITCH_BTNS_BASE_ADDR)

unsigned *switchData = SWITCH_BTNS_REG;
unsigned *switchTri = SWITCH_BTNS_REG + 1; 

#define RGB_RED 04444
#define RGB_GREEN 02222


unsigned *rgbLEDsData = RGB_LEDS_REG;
unsigned *rgbLEDsTri  = RGB_LEDS_REG + 1;


#define CLK_FREQ 83333333 // FCLK0 frequency not found in xparameters.h

PmodMAXSONAR leftSonar;

// Car states
typedef enum
{
    LEFT,
    RIGHT,
    IDLE
} Previous_Direction_T;

typedef enum
{
    STOP,
    GO,
    INIT
}Previous_State_T;


Previous_State_T previous_state = INIT;
Previous_Direction_T previous_direction = IDLE; // Shared resource. Mutex driven.
// SemaphoreHandle_t state_mutex;


// TaskHandle_t sonarTaskHandle = NULL;

void delay_ms(int ms);
void DHB1_GPIO_init(XGpio *InstancePtr,
                    u32 GPIO_BASEADDR)
{
  InstancePtr->BaseAddress = GPIO_BASEADDR;
  InstancePtr->InterruptPresent = 0;
  InstancePtr->IsDual = 1;
  InstancePtr->IsReady = XIL_COMPONENT_IS_READY;
}

// int leftMotorSpeed = 

MotorFeedback motor1Feedback; 

int main()
{
  // Initialize GPIO interface
    DHB1_GPIO_init(&DHB1_GPIO,
                 XPAR_PMOD_DHB1_0_GPIO_BASEADDR);

    XGpio_SetDataDirection(&DHB1_GPIO, M1_CHANNEL, 0xC);
    XGpio_SetDataDirection(&DHB1_GPIO, M2_CHANNEL, 0xC);
    xil_printf("check 1\r");

    // Initialize motor instance
    DHB1_begin(&motor,
                XPAR_PMOD_DHB1_0_GPIO_BASEADDR,
                XPAR_PMOD_DHB1_0_PWM_BASEADDR,
                PMOD_DHB1_CLOCK_FREQ_HZ,
                PWM_PERIOD * 3);
    xil_printf("check 2\r");

    XGpio_Initialize(&LS1_GPIO, PMOD_LS1_BASEADDR);
    XGpio_SetDataDirection(&LS1_GPIO, 2, 0xFF);

    //volatile u32 *SensorData = (u32 *)PMOD_LS1_BASEADDR + XGPIO_DATA_OFFSET;
	// volatile u32 *SensorTristateReg = (u32 *)PMOD_LS1_BASEADDR + XGPIO_TRI_OFFSET;
    // *SensorTristateReg = 0xF;

    // Motor Driver PWM
    u32 PWM_ctrl_reg;
    u32 PWM_status_reg;
    u32 PWM_period_reg;
    u32 PWM_duty_reg;

    u32 MotorFeedback_reg1;
    u32 MotorFeedback_reg2;

    // PWM_Set_Duty(XPAR_PMOD_DHB1_0_PWM_BASEADDR, PWM_PERIOD * 4, 0);
    // PWM_Set_Duty(XPAR_PMOD_DHB1_0_PWM_BASEADDR, PWM_PERIOD * 4, 1);

    *rgbLEDsTri = 0x0;
	// Sonar example
	MAXSONAR_begin(&leftSonar, PMOD_SONAR0_BASEADDR, CLK_FREQ);

    u32 dist1;
    // Driver foward for approx 15 seconds
    //DHB1_setDirs(&motor, 0, 0);
    DHB1_motorEnable(&motor);
    ///DHB1_setMotorSpeeds(&motor, 50, 50);
    //DHB1_turn(&motor, 0, 20);  
    //MotorFeedback_init(&motor1Feedback, u32 baseAddr, u32 clkFreqHz, u32 edgesPerRev, u32 gearboxRatio);

    xil_printf("check 3\r");

    u32 m1, m2, count = 0;
    while (1)
    {

        dist1 = MAXSONAR_getDistance(&leftSonar);

        if(*switchData & (1 << 0)){


            //xil_printf("left = %3d", dist1);
            if(dist1 < 6){
                DHB1_setMotorSpeeds(&motor, 0, 0);
                *rgbLEDsData = RGB_RED;
                previous_state = STOP;


            }
            else{

                *rgbLEDsData = RGB_GREEN;
                // if(previous_state == STOP){
                //     DHB1_setMotorSpeeds(&motor, 45, 45);
                // }
                previous_state = GO;
                //DHB1_setMotorSpeeds(&motor, 80,80);
               if(XGpio_DiscreteRead(&LS1_GPIO, 2) == BOTH_SENSORS){
                    DHB1_setDirs(&motor,0, 0);
                    DHB1_setMotorSpeeds(&motor, 55, 55);
                    previous_direction = IDLE;
                   xil_printf("BOTH");
               }

                else if (XGpio_DiscreteRead(&LS1_GPIO, 2) & L_SENSOR){
                        //xil_printf("left: \r");
                        //xil_printf("0x%80x",XGpio_DiscreteRead(&LS1_GPIO,2));
                        

                    previous_direction = LEFT;
                    xil_printf("Previous Direction: LEFT");
                    xil_printf("\n");
                    DHB1_turn(&motor, 1, 50);
                        
                }
                else if (XGpio_DiscreteRead(&LS1_GPIO, 2) & R_SENSOR){
                        //xil_printf("right: \r");
                        //xil_printf("0x%80x",XGpio_DiscreteRead(&LS1_GPIO,2));
                    xil_printf("Previous Direction: RIGHT");
                    xil_printf("\n");
                    previous_direction = RIGHT;
                    DHB1_turn(&motor, 0, 50);
                } 
                else {
                    if(previous_direction = LEFT){
                        DHB1_turn(&motor,0,50);
                    }
                    else if(previous_direction = RIGHT){
                        DHB1_turn(&motor,1, 50);
                    }
                    else{
                        DHB1_setMotorSpeeds(&motor, 0, 0);
                    }
                   

                }

                // else if (XGpio_DiscreteRead(&LS1_GPIO, 2) & L_SENSOR){
                //     xil_printf("left: \r");
                //     xil_printf("0x%80x",XGpio_DiscreteRead(&LS1_GPIO,2));
                //     xil_printf("\n");

                //     DHB1_turn(&motor, 1, 50);
                // }
                // else if (XGpio_DiscreteRead(&LS1_GPIO, 2) & R_SENSOR){
                //     xil_printf("right: \r");
                //     xil_printf("0x%80x",XGpio_DiscreteRead(&LS1_GPIO,2));
                //     xil_printf("\n");

                //     DHB1_turn(&motor, 0, 50);
                // }
               

            //    }    else{
            //         DHB1_setMotorSpeeds(&motor,0,0);
            //     }
      


            }
        }
        else{
            *rgbLEDsData = 0x0;
            DHB1_setMotorSpeeds(&motor, 0, 0);
        }

        

        // PWM data
        // PWM_ctrl_reg = PWM_mReadReg(
        //     XPAR_PMOD_DHB1_0_PWM_BASEADDR,
        //     PWM_CTRL_REG_OFFSET);
        // PWM_status_reg = PWM_mReadReg(
        //     XPAR_PMOD_DHB1_0_PWM_BASEADDR,
        //     PWM_CTRL_REG_OFFSET);
        // PWM_period_reg = PWM_Get_Period(XPAR_PMOD_DHB1_0_PWM_BASEADDR);
        // PWM_duty_reg = PWM_Get_Duty(XPAR_PMOD_DHB1_0_PWM_BASEADDR, 0);
        // MotorFeedback_reg1 = MOTORFEEDBACK_mReadReg(XPAR_PMOD_DHB1_0_GPIO_BASEADDR,0x04 );
        // MotorFeedback_reg2 = MOTORFEEDBACK_mReadReg(XPAR_PMOD_DHB1_0_GPIO_BASEADDR,0x08 );

        // xil_printf("MOTOR 1 FEEDBACK: 0x%08x\r", MotorFeedback_reg1);
        // xil_printf("MOTOR 2 FEEDBACK: 0x%08x\r", MotorFeedback_reg2);
        // xil_printf("PWM Control: 0x%08x\r", PWM_ctrl_reg);
        // xil_printf("PWM Status: 0x%08x\r", PWM_status_reg);
        // xil_printf("PWM Period: 0x%08x\r", PWM_period_reg);
        // xil_printf("PWM Duty: 0x%08x\r", PWM_duty_reg);

        // // Motor data
        // m1 = XGpio_DiscreteRead(&DHB1_GPIO, M1_CHANNEL);
        // m2 = XGpio_DiscreteRead(&DHB1_GPIO, M2_CHANNEL);
        //MotorFeedback_getSpeeds(MotorFeedback *motorFeedback, int *motor_speed);
        //xil_printf("0x%08x, 0x%08x\r", m1, m2);
        // delay_ms(3000);
        // count++;

        // if (count >= 5)
        // break;
    }

    // Stop motor
    DHB1_motorDisable(&motor);

    return 0;
}

void delay_ms(int ms)
{
  for (int i = 0; i < 1500 * ms; i++)
    asm("nop");
}
