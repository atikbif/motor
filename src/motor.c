#include "motor.h"
#include "definitions.h"
#include "motorcontrolsettings.h"

#include "stm32f0xx_conf.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "settings.h"

volatile unsigned char zcfound;
volatile unsigned char phase;
volatile unsigned char autostep;
volatile unsigned char risingdelay;
volatile unsigned char fallingdelay;
volatile unsigned char risingedge;
volatile unsigned char startstate;
volatile unsigned char run;
volatile unsigned char ontimesample;

volatile unsigned short commcounter;
volatile unsigned short step;
volatile unsigned short bemfsample;
volatile unsigned short dcbussample;
volatile unsigned short commcounter;
volatile unsigned short demagcounter;
volatile unsigned short zccounter;
volatile unsigned short commthreshold;
volatile unsigned short demagthreshold;
volatile unsigned short runningdc;
volatile unsigned short potvalue;
volatile unsigned short zcthreshold;
volatile unsigned short dutycyclehold;

volatile unsigned long holdcounter;
volatile unsigned long alignmentcounter;
volatile unsigned long rampspeed;
volatile unsigned long bemfchannel;

volatile unsigned short curStep = 0;
volatile unsigned short prevStep = 0;

#define FILTER_CNT     4

volatile unsigned short step_filter[FILTER_CNT]={0,0,0,0};
volatile unsigned char filter_cnt = 0;

unsigned char polepairs = 7;
unsigned short alignmentdc = 100;
unsigned short holdrpm = 500;
unsigned short startuprpmpersecond = 100;

extern unsigned short settings[SETTINGS_CNT];

volatile unsigned long long0;
volatile unsigned short minstep;
unsigned short rpmGoal = 0;



void TIM1_CC_IRQHandler (void);

void commutate(void);
void commutate2(void);
void motorstartinit(void);
void delay( unsigned long time);
void flashled(unsigned char flashes);
void selftest(void);
unsigned short readadc( unsigned char chnl);
void pwmisr(void);

void TIM1_CC_IRQHandler (void)
{
    pwmisr();
}

void commutate(void)
{

    switch(phase)
    {
        case 0: // phase AB
            TIM1->CCER = b3+b0  +b7+b4 +b8; //
            TIM1->CCMR1= 0x4868 +b15 + b7; // force B active low
            TIM1->CCMR2= 0x6868 +b7;
            break;
        case 1: // phase AC
            TIM1->CCER = b3+b0  +b4  +b8+b11; //
            TIM1->CCMR1= 0x6868 +b15 + b7;
            TIM1->CCMR2= 0x6848 +b7; // force C active low
            break;
        case 2: // phase BC
            TIM1->CCER = b4+b7  +b8+b11  +b0;
            TIM1->CCMR1= 0x6868 +b15 + b7;
            TIM1->CCMR2= 0x6848 +b7; // force C active low
            break;
        case 3: // phase BA
            TIM1->CCER = b4+b7 +b3+b0 +b8;
            TIM1->CCMR1= 0x6848 +b15 + b7; // force A active low
            TIM1->CCMR2= 0x6868 +b7;
            break;
        case 4: // phase CA
            TIM1->CCER = b8+b11 +b3+b0 +b4;
            TIM1->CCMR1= 0x6848 +b15 + b7; // force A active low
            TIM1->CCMR2= 0x6868 +b7;
            break;
        case 5: // phase CB
            TIM1->CCER = b8+b11 +b4+b7 +b0;
            TIM1->CCMR1= 0x4868 +b15 + b7; // force B active low
            TIM1->CCMR2= 0x6868 +b7;
            break;
    } // end of phase switch statement
} // end of commutate function

void commutate2(void)
{

    //TIM1->CCER=b10+b8+b6+b4+b2+b0; // enable all 6
    switch(phase)
    {
        case 0: // phase AB
            // enable all 6 except AN
            // invert AN
            TIM1->CCER=b10+b8+b6+b4+b0  +b3;
            TIM1->CCMR1=0x4868+b15+b7; // B low, A PWM
            TIM1->CCMR2= 0x6858 +b7; // force C ref high (phc en low)
            break;
        case 1: // phase AC
            // enable all 6 except AN
            // invert AN
            TIM1->CCER=b10+b8+b6+b4+b0  +b3;
            TIM1->CCMR1=0x5868+b15+b7; // force B high and A PWM
            TIM1->CCMR2= 0x6848 +b7; // force C ref low
            break;
        case 2: // phase BC
            // enable all 6 except BN
            // invert BN
            TIM1->CCER=b10+b8+b4+b2+b0 +b7;
            TIM1->CCMR1=0x6858+b15+b7; // force B PWM and A high
            TIM1->CCMR2= 0x6848 +b7; // force C ref low
            break;
        case 3: // phase BA
            // enable all 6 except BN
            // invert BN
            TIM1->CCER=b10+b8+b4+b2+b0 +b7;
            TIM1->CCMR1=0x6848+b15+b7; // force B PWM and A ref low
            TIM1->CCMR2= 0x6858 +b7; // force C ref high
            break;
        case 4: // phase CA
            // enable all 6 except CN
            // invert CN
            TIM1->CCER=b8+b6+b4+b2+b0 +b11; // enable all 6 except CN
            TIM1->CCMR1=0x5848+b15+b7; // force B high and A ref low
            TIM1->CCMR2= 0x6868 +b7; // force C PWM
            break;
        case 5: // phase CB
            // enable all 6 except CN
            // invert CN
            TIM1->CCER=b8+b6+b4+b2+b0 +b11; // enable all 6 except CN
            TIM1->CCMR1=0x4858+b15+b7; // force B low and A high
            TIM1->CCMR2= 0x6868 +b7; // force C PWM
            break;
    } // end of phase switch statement
} // end of commutate2 function


// pwmisr runs just at the end of each PWM cycle
void pwmisr(void)
{
    //**********************************************************

    ADC1->CHSELR = bemfchannel; // set ADC MUX to proper bemf channel
    ADC1->CR = b2;  // start adc conversion
    // housekeeping increment of some timers while adc is converting
    zccounter++;  // housekeeping increment of some timers while adc is converting
    alignmentcounter++;
    holdcounter++;
    while((ADC1->CR & b2)==b2) ; // wait for conversion to complete
    bemfsample= ADC1->DR;

    //DAC->DHR12R1 = bemfsample; // put out for diagnostic purposes

    // autostep is true during ramping up
    if(autostep)
    {
        commcounter++;
        if(commcounter>step)
        {
            commcounter=0;
            phase++;
            if(phase>5) phase=0;
            commutate2();

        }
    } // end of if(autostep)

    if(run==0) {
        startstate=0;
        settings[0] = 0;
        for(filter_cnt=0;filter_cnt++;filter_cnt<FILTER_CNT) step_filter[filter_cnt]=0;
        filter_cnt = 0;
    }

    switch(startstate)
    {
        case 0:
            TIM1->CCER = 0;
            if(run)
            {
                motorstartinit();
                startstate=5;
            }
            break;

        case 5: // setup alignment
            TIM1->CCR1= alignmentdc;
            TIM1->CCR2= alignmentdc;
            TIM1->CCR3= alignmentdc;
            phase=0;
            commutate2();
            alignmentcounter=0;
            startstate=10;
            break;

        case 10: // timing out alignment
            if(alignmentcounter>alignmenttime)
            {
                rampspeed=1;
                commcounter=0;
                autostep=255;
                TIM1->CCR1= rampupdc;
                TIM1->CCR2= rampupdc;
                TIM1->CCR3= rampupdc;
                startstate=20;
            }
            break;

        case 20: // ramping up
            rampspeed = rampspeed+rampuprate; // accelerate
            long0 = 4000000000ul;
            long0 = long0/rampspeed; // step time = constant/speed
            if(long0>30000) long0=30000;
            step=long0;
            if(step<=minstep)
            {
                holdcounter=0;
                startstate=100;
            }
            if(filter_cnt<FILTER_CNT) {
               step_filter[filter_cnt++] = step;
            }
            break;

        case 100: // wait for hold time, holding at speed to allow sync
            if(holdcounter>holdtime) startstate=110;
            break;

        case 110: // wait to get into phase 5
            if(phase==5) startstate=120;
            break;

        case 120: // wait for leading edge of phase 0 (commutation)
            if(phase==0)
            {
                demagcounter=0;
                demagthreshold = (step*demagallowance)>>8;
                startstate=130;
            }
            break;

        case 130: // wait out demag time (for current in phase to go to 0)
            demagcounter++;
            if(demagcounter>demagthreshold)
            {
                startstate=140;
            }
            break;

        case 140: // looking for zero crossing of bemf
            if( zcfound && (zccounter>maxstep) )
            {
                startstate=0;
                break;
            }


            if(risingedge)
            {
                if((bemfsample>zcthreshold)||(zccounter>maxstep) )
                {
                    if(zcfound){ if(prevStep) {step = (zccounter + prevStep)/2;}else {step = zccounter;} prevStep = zccounter; }
                    else prevStep = 0;


                    commthreshold = (((unsigned short)(step*1))*risingdelay)>>8;
                    zccounter=0;
                    commcounter=0;
                    startstate=150;
                    risingedge=0;
                    zcfound=255;
                    autostep=0;
                }
            }
            else
            {
                if((bemfsample<zcthreshold)||(zccounter>maxstep) )
                {
                    if(zcfound){ if(prevStep) {step = (zccounter + prevStep)/2;}else {step = zccounter;} prevStep = zccounter; }
                    else prevStep = 0;


                    commthreshold = (((unsigned short)(step*1))*risingdelay)>>8;
                    zccounter=0;
                    commcounter=0;
                    startstate=150;
                    risingedge = 255;
                    zcfound=255;
                    autostep=0;
                }
            }
            break;

        case 150:  // wait out commutation delay (nominal 1/2 step time)
            TIM1->CCR1= runningdc;
            TIM1->CCR2= runningdc;
            TIM1->CCR3= runningdc;
            commcounter++;
            if(commcounter>commthreshold)
            {
                if(filter_cnt<FILTER_CNT) {
                   step_filter[filter_cnt++] = step;
                }
                phase++; // commutate
                if(phase>5)
                {
                    phase=0;
                }
                commutate2();
                demagcounter=0;
                demagthreshold = (step*demagallowance)>>8;
                startstate=130;  // go back to wait out demag
                if(dutycyclehold) dutycyclehold--;
            }
            break;
    } // end of startstate state machine






    ADC1->CHSELR = b2; // set ADC MUX to pot channel (ain2)
    ADC1->CR = b2;  // start adc conversion

    // switch statement determines next bemf ADC input channel
    switch(phase)
    {
        case 0: // ab
            bemfchannel= 1<<8 ; // read phase c
            break;
        case 1: // ac
            bemfchannel= 1<<7 ; // read phase b
            break;
        case 2:  // bc
            bemfchannel= 1<<6 ; // read phase a
            break;
        case 3:  // ba
            bemfchannel= 1<<8 ; // read phase c
            break;
        case 4:  // ca
            bemfchannel= 1<<7 ; // read phase b
            break;
        case 5:  // cb
            bemfchannel= 1<<6 ; // read phase a
            break;

    } // end of phase switch statement


    if(ontimesample)
    {
        GPIOC->MODER = 0x54000000; // C15,14,13 PP to turn on attenuator
        TIM1->CCR4 = 100; // move isr to halfway thry on time
        //zcthreshold = 620;  // for 12V bus
        long0 = dcbussample;
        long0 = long0 * 599;
        long0 = long0>>10;
        zcthreshold = long0;
    }
    else
    {
        GPIOC->MODER = 0; // turn attenuator off
        TIM1->CCR4 = 1100;
        zcthreshold = 200;  // threshold to about zero
    }

    while((ADC1->CR & b2)==b2) ; // wait for conversion to complete
    potvalue=  ADC1->DR;
    //if(potvalue>4050) ledon else ledoff

    ADC1->CHSELR = b9; // set ADC MUX to dc bus channel (ain9)
    ADC1->CR = b2;  // start adc conversion
    while((ADC1->CR & b2)==b2) ; // wait for conversion to complete
    dcbussample =  ADC1->DR;

    DAC->DHR12R1 = dcbussample; // put out for diagnostic purposes


    //**************************************************************

    // clear interrupt
    TIM1->SR &= ~0x01F;
    NVIC_ClearPendingIRQ(14);
}





void  motorstartinit(void)
{
    TIM1->CCER = 0;

    // b12 to enable brk input
    // b13 for break polarity
    // (b15+b11);  //  set MOE

    TIM1->BDTR= b15+b11+b10+b12+b13;  //  set MOE


    // ADC count used as zero crossing threshold
    // may need to be set higher/lower for higher/lower bemf motors
    // setting too low may cause noise sensitivity issues
    zcthreshold = 200 ;


    ontimesample=0;
    phase = 0;
    holdcounter=0;
    startstate=0;
    commcounter=0;
    step=3670; // unit is 50uS pwm periods
    autostep = 0;
    risingedge = 0;
    zcfound=0;
    alignmentcounter=0;
    rampspeed=1;
    commthreshold=0;

    // risingdelay and fallingdelay control the time delay waited
    // between zero crossing detection and commutation.  The nominal
    // value is 128 which means 128/256 or 1/2 of a step time, which
    // is the phase shift between line to neutral zero crossing
    // (what is actually detected) and line to line zero cross
    // (nominal correct commutation time)
    // adjustment will advance or delay commutation (phase advance)
    risingdelay=128;
    fallingdelay=128;

    // sets open loop motor voltage (speed) 1200 for 100% duty cycle
    runningdc = alignmentdc;

    dutycyclehold=100;

} // end of motor start init function



void delay( unsigned long time)  // 1000 is 200 usec
{
    while(time>0) time--;
}



void selftest(void)
{

    unsigned short word0;


    // port B setup
    GPIOB->ODR = 0;
    /*
    15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    pp pp pp pp in pp pp pp pp pp pp pp pp pp an an
    01 01 01 01 00 01 01 01 01 01 01 01 01 01 11 11
      5     5     1     5     5     5     5     F
    */
    GPIOB->MODER = 0x5515555F;


    //test for jumper from P6-2(pb10) to P6-1(pb11)and return from selftest if not
    GPIOB->ODR &= ~b10;
    delay(1000);
    if((GPIOB->IDR & b11) != 0) return;

    GPIOB->ODR |= b10;
    delay(1000);
    if((GPIOB->IDR & b11) != b11) return;

    GPIOB->ODR  &= ~b10;
    delay(1000);
    if((GPIOB->IDR & b11) != 0) return;

    GPIOB->ODR  |= b10;
    delay(1000);
    if((GPIOB->IDR & b11) != b11) return;

    // program is in test mode

    // port A setup
    GPIOA->ODR = 0;
    /*
    15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    in af af in pp pp pp pp an an an an an an an an
    00 10 10 00 01 01 01 01 11 11 11 11 11 11 11 11
      2     8     5     5     F     F     F     F
    */
    GPIOA->MODER = 0x2855FFFF;


    // comparator setup
    COMP->CSR = 0;
    #ifdef currentlimitenable
    COMP->CSR = COMP->CSR + b27+b16+b22+b20+b25+b24;  // cy-by-cy active
    #endif
    #ifdef overcurrentenable
    COMP->CSR = COMP->CSR + b0+b6+b5+b11+b8; //  brk active
    #endif

    // ADC setup
    ADC1->CR= b31 ; // start ADC calibration
    while( ADC1->CR & b31);  // wait for calibration to complete
    while( ADC1->CR & b31);  // wait for calibration to complete
    ADC1->CR = 1; // enable ADC
    ADC1->CFGR1 = b16; // enable discontinuous mode
    ADC1->SMPR = 1;


    // routine to manually test pot and LED
    while(1)
    {

        potvalue = 4095 - readadc(2);

        switch((potvalue>>9))
        {
            case 0:ledoff;break;
            case 1:ledon;break;
            case 2:ledoff;break;
            case 3:ledon;break;
            case 4:ledoff;break;
            case 5:ledon;break;
            case 6:ledoff;break;
            case 7:ledon;break;
            case 8:break;
        }  // end of pot test case statement

        //test for jumper from P6-2(pb10) to P6-1(pb11)and break out of loop if not
        GPIOB->ODR &= ~b10;
        delay(1000);
        if((GPIOB->IDR & b11) != 0) break;

        GPIOB->ODR |= b10;
        delay(1000);
        if((GPIOB->IDR & b11) != b11) break;

        GPIOB->ODR  &= ~b10;
        delay(1000);
        if((GPIOB->IDR & b11) != 0) break;

        GPIOB->ODR  |= b10;
        delay(1000);
        if((GPIOB->IDR & b11) != b11) break;

    } // end of pot test endless loop

    ledoff;
    delay(19999999);

    // now test DC bus sensing
    word0 = readadc(9); // read bemfa
    if(word0<1430) flashled(11);
    if(word0>1748) flashled(12);

    // now test bridge outputs and bemf sensing
    phaseadisable;
    phasebdisable;
    phasecdisable;

    phaseahigh;
    phaseblow;
    phaseclow;

    // enable A and set high.  disable B and C
    phaseaenable;
    delay(500);  // wait 100 usec for settling

    word0 = readadc(6); // read bemfa
    if(word0<2048) flashled(13);  // fault if pha not high

    word0 = readadc(7); // read bemfb
    if(word0<2048) flashled(14);  // fault if phb not high

    word0 = readadc(8); // read bemfc
    if(word0<2048) flashled(15);  // fault if phc not high

    // now enable B low
    phasebenable;
    delay(500);  // wait 100 usec for settling
    word0 = readadc(7); // read bemfb
    phasebdisable;
    if(word0>2048) flashled(21);  // fault if phb not low

    // now enable C low
    phasecenable;
    delay(500);  // wait 100 usec for settling
    word0 = readadc(8); // read bemfc
    phasecdisable;
    if(word0>2048) flashled(22);  // fault if phc not low

    phaseadisable;
    phasebenable;
    phasebhigh;

    // B is high, A and C disabled
    delay(500);  // wait 100 usec for settling
    word0 = readadc(6); // read bemfa
    if(word0<2048) flashled(23);  // fault if pha not high

    // disable verified for all three phases

    phaseadisable;
    phasebdisable;
    phasecdisable;

    // activate phase AB
    phaseahigh;
    phaseblow;
    phaseaenable;
    phasebenable;
    delay(500);  // wait 100 usec for settling
    word0 = readadc(6); // read bemfa
    if(word0<2048) flashled(24);  // fault if pha not high
    word0 = readadc(7); // read bemfb
    if(word0>2048) flashled(24);  // fault if phb not low

    // activate phase BA
    phasebhigh;
    phasealow;
    delay(500);  // wait 100 usec for settling
    word0 = readadc(6); // read bemfa
    if(word0>2048) flashled(25);  // fault if pha not low
    word0 = readadc(7); // read bemfb
    if(word0<2048) flashled(25);  // fault if phb not high

    // activate phase AC
    phasebdisable;
    phaseahigh;
    phaseclow;
    phasecenable;
    delay(500);  // wait 100 usec for settling
    word0 = readadc(6); // read bemfa
    if(word0<2048) flashled(31);  // fault if pha not high
    word0 = readadc(8); // read bemfc
    if(word0>2048) flashled(31);  // fault if phc not low

    // activate phase CA
    phasechigh;
    phasealow;
    delay(500);  // wait 100 usec for settling
    word0 = readadc(6); // read bemfa
    if(word0>2048) flashled(32);  // fault if pha not low
    word0 = readadc(8); // read bemfc
    if(word0<2048) flashled(32);  // fault if phc not high

    // activate phase BC
    phaseadisable;
    phasebenable;
    phasebhigh;
    phaseclow;
    delay(500);  // wait 100 usec for settling
    word0 = readadc(7); // read bemfb
    if(word0<2048) flashled(33);  // fault if phb not high
    word0 = readadc(8); // read bemfc
    if(word0>2048) flashled(33);  // fault if phc not low

    // activate phase CB
    phasechigh;
    phaseblow;
    delay(500);  // wait 100 usec for settling
    word0 = readadc(7); // read bemfb
    if(word0>2048) flashled(34);  // fault if phb not low
    word0 = readadc(8); // read bemfc
    if(word0<2048) flashled(34);  // fault if phc not high

    // all 6 phases have been tested (voltage sensing)

    phaseadisable;
    phasebdisable;
    phasecdisable;

    // activate phase AB  (should not trip)
    phaseahigh;
    phaseblow;
    phaseaenable;
    phasebenable;
    delay(500);  // wait 100 usec for settling
    if(COMP->CSR & b30) flashled(35); // comp2 at 2.24A threshold

    // now turn on C lower (should trip)
    phaseclow;
    phasecenable;
    delay(500);  // wait 100 usec for settling
    if((COMP->CSR & b30)==0) flashled(41); // comp2 at 2.24A threshold

    // overcurrent (higher threshold) should be low
    if((COMP->CSR & b14)) flashled(42); // comp1  high threshold


    phaseadisable;
    phasebdisable;
    phasecdisable;
    ledon;

    while(1)
    {
        word0=0;
        // break out of loop when jumper restored
        GPIOB->ODR &= ~b10;
        delay(1000);
        if((GPIOB->IDR & b11) != 0) word0=0;
        else word0++;
        GPIOB->ODR |= b10;
        delay(1000);
        if((GPIOB->IDR & b11) != b11) word0=0 ;
        else word0++;
        GPIOB->ODR  &= ~b10;
        delay(1000);
        if((GPIOB->IDR & b11) != 0) word0=0 ;
        else word0++;
        GPIOB->ODR  |= b10;
        delay(1000);
        if((GPIOB->IDR & b11) != b11) word0=0;
        else word0++;
        if(word0==4) break;
    }
    ledoff;

    // jumper has been added across phase B resistor
    delay(5000000); //delay 1 second

    // test for overcurrent trip true
    phasebhigh;
    phasealow;
    phaseclow;
    phaseaenable;
    phasecenable;
    phasebenable;
    delay(500);
    if((COMP->CSR & b14)==0) flashled(43); // comp1, error if no overload


    phaseadisable;
    phasebdisable;
    phasecdisable;

    delay(10000000); //delay 2 second

    ledon;


    while(1);

}  // end of selftest



void flashled(unsigned char flashes)
{
    unsigned char count;
    unsigned char tens;
    unsigned char ones;

    phaseadisable;
    phasebdisable;
    phasecdisable;


    tens = flashes/10;
    ones = flashes - (tens*10);

    while(1)
    {
        ledoff;
        delay(8000000);
        count=0;
        while(1)
        {
            ledon;
            delay(1000000);
            ledoff;
            delay(1000000);
            count++;
            if(count==tens) break;
        }

        count=0;
        delay(1300000);
        while(1)
        {
            ledon;
            delay(1000000);
            ledoff;
            delay(1000000);
            count++;
            if(count==ones) break;
        }

    } // end of main flashled endless loop

} // end of flashled function



unsigned short readadc( unsigned char chnl)
{

    ADC1->CHSELR = (1<<chnl); // set ADC MUX
    ADC1->CR = b2;  // start adc conversion

    while((ADC1->CR & b2)==b2) ; // wait for conversion to complete
    return ADC1->DR;
} // end of readadc function

void MotorTask( void *pvParameters )
{
    unsigned short rpmGoalCnt = 0;
    unsigned char tmp=0;
    unsigned long stepSum = 0;

    portTickType xLastExecutionTime;
    xLastExecutionTime = xTaskGetTickCount();

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG | RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    //**********************************************

    vTaskDelayUntil( &xLastExecutionTime, ( portTickType ) 1000 / portTICK_RATE_MS );   //let power supply settle

    selftest();

    // port A setup
    //GPIOA->MODER = 0x2A2AFFF0; // a5,a4,3,2,6,7 anlg, a8,9,10,12 AF
    GPIOA->MODER = 0x2A6AFFF0; // 11 pp, a5,a4,3,2,6,7 anlg, a8,9,10,12 AF
    ledoff;
    //GPIOA->AFRH  = 0x00020222; // pa8,9,10,12 AF2 for timer
    GPIOA->AFR[1] = 0x00020222; // pa8,9,10,12 AF2 for timer

    // port B setup
    // b22 PB11 as output
    GPIOB->MODER = 0xA800000F+b22; // b1,0 anlg, b15,14,13 AF
    //GPIOB->AFRH = 0x22220000;  // b15,14,13,12 AF2 for timer
    GPIOB->AFR[1] = 0x22220000;  // b15,14,13,12 AF2 for timer

    // port C setup
    GPIOC->ODR = 0;


    // ADC setup
    ADC1->CR= b31; // start ADC calibration
    while( ADC1->CR & b31);  // wait for calibration to complete
    while( ADC1->CR & b31);  // wait for calibration to complete
    ADC1->CR = 1; // enable ADC
    ADC1->CFGR1 = b16; // enable discontinuous mode
    ADC1->SMPR = 1;



    // comparator setup
    // (b25+b24) send comp2 output to tim1 OCref clear
    // b27 invert comp2 output
    // b16 enable comp2
    // b22+b20 select PA5 as inverting input for comp2

    // b0 enable comp1
    // (b6+b5) select PA0 comp1 inverting input
    // b8 send comp1 output to tim1 break
    // b11 to invert comp1 output

    COMP->CSR = 0;
    #ifdef currentlimitenable
    COMP->CSR = COMP->CSR + b27+b16+b22+b20+b25+b24;  // cy-by-cy active
    #endif
    #ifdef overcurrentenable
    COMP->CSR = COMP->CSR + b0+b6+b5+b11+b8; //  brk active
    #endif


    // tim1 setup
    TIM1->PSC = 1;
    TIM1->SMCR = b15+b4+b5+b6; // make ETR (external trigger) input active low
    TIM1->CR2= 0;
    TIM1->CCR1= 200;
    TIM1->CCR2= 200;
    TIM1->CCR3= 200;
    TIM1->CCR4= 1100;
    TIM1->ARR=1200;
    TIM1->CR1=0x0001;


    // note: b15 b7 and b7 are to enable ETR  based current limit
    TIM1->CCMR1= 0x6868 +b15 + b7;
    TIM1->CCMR2= 0x6868 +b7;
    // b4 for cc4 and b7 for brk interrupt
    //TIM1->DIER = b4+b7;  // enable cc4 interrupt
    TIM1->DIER = b4+b6;

    // DAC setup
    DAC->CR = 1; // enable DAC


    // set up interrupts
    //NVIC_EnableIRQ(14);
    //__enable_irq();

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    // initialization
    run=0;
    motorstartinit();

    clearmark;


    while(1)
    { // backgroung loop

        if( (TIM1->BDTR & b15)==0) // overcurrent fault condition
        {
            run=0;
            while(1)
            {
                ledon;
                vTaskDelayUntil( &xLastExecutionTime, ( portTickType ) 200 / portTICK_RATE_MS);
                ledoff;
                vTaskDelayUntil( &xLastExecutionTime, ( portTickType ) 200 / portTICK_RATE_MS);
                if( (4095-potvalue)<100)
                {
                    TIM1->BDTR= b15+b11+b10+b12+b13;  //  set MOE
                    break;
                }


            } //end of LED flash loop

        } // end of if overcurrent




        //TIM1->BDTR= b15+b11+b10+b12+b13;  //  set MOE


        //if(potvalue<(4095-200)) run=255;
        //else if(potvalue>(4095-100)) run=0;



        if(rpmGoal == 0) {
            if( (4095ul-potvalue)>200) run=255;
            if( (4095ul-potvalue)<100) run=0;

            runningdc = ((4095-potvalue)*1200)>>12;
            if(runningdc>1195) runningdc=1300;
        }else {
            run = 255;
            //if(runningdc==0) runningdc = 100;
            if(startstate>=120) {
                rpmGoalCnt++;
                if(rpmGoalCnt>=1) {
                    rpmGoalCnt = 0;
                    if((settings[0]<rpmGoal)&&(runningdc<1100)&&(((rpmGoal-settings[0])*100)/rpmGoal>=5)) runningdc++;
                    else if((settings[0]>rpmGoal)&&(runningdc>10)&&(((settings[0]-rpmGoal)*100)/rpmGoal>=5)&&(runningdc>1)) runningdc--;
                }
            }
        }

        if(run) ledon;
        if(run==0) ledoff;


        if(dutycyclehold && (runningdc>600)) runningdc=600;

        #ifdef ontimesampleenable
        if(runningdc>800) ontimesample=255;
        else ontimesample=0;
        #endif

        //if(COMP->CSR & b30) // comp2
        if(COMP->CSR & b14)  // comp1
        {
            setmark;
        }
        else
        {
            //clearmark;
        }

        if(filter_cnt>=FILTER_CNT) {
            stepSum = 0;
            for(tmp=0;tmp<FILTER_CNT;tmp++) {stepSum+=step_filter[tmp];}
            curStep = stepSum/FILTER_CNT;
            filter_cnt = 0;
            settings[0]=200000ul/(polepairs*curStep);
        }

        //DAC->DHR12R1 = potvalue; // put out for diagnostic purposes
        vTaskDelayUntil( &xLastExecutionTime, Motor_DELAY);
        //GPIOA->ODR ^= GPIO_Pin_11;

    } // end of backgroung loop
}
