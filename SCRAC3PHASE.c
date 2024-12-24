 /*
 * File:   SCRbasedVol.c
 * Author: Sourav Sadhukhan
 * Application: SCR based Dc voltage regulation from AC 3 ph
 * Created on 8 March, 2019, 12:38 PM
 */
#pragma config FOSC = INTRC_NOCLKOUT    // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF               // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON               // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON               // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = ON                  // Code Protection bit (Program memory code protection is enabled)
#pragma config CPD = ON
#pragma config BOREN = ON               // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = OFF               // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF              // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V           // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT = OFF                // Flash Program Memory Self Write Enable bits (Write protection off)

#define _XTAL_FREQ 8000000

#define SegPort PORTB
#define comh    PORTCbits.RC7 //p18
#define comt    PORTCbits.RC6 //p17
#define comu    PORTCbits.RC5 //p16
#define rzcd    PORTCbits.RC2 //p13
#define yzcd    PORTCbits.RC3 //p14
#define bzcd    PORTCbits.RC4 //p15
#define thy1G   PORTAbits.RA2 // p4 r+
#define thy2G   PORTAbits.RA3 // p5 b-
#define thy3G   PORTAbits.RA4 // p6 y+
#define thy4G   PORTAbits.RA5 // p7 r-
#define thy5G   PORTAbits.RA6 // p10 b+
#define thy6G   PORTAbits.RA7 // p9 y-
//adc voltage ch0 p2//adc current ch1 p3

#include <xc.h>
#include <pic16f886.h>
__EEPROM_DATA(0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF); 

bit ph,pt,pu,flag,thy1GF,thy2GF,thy3GF,thy4GF,thy5GF,thy6GF;
char dh,dt,du,digitcount,triggerCon[2],triggerConIndex,triggerTimeCon[2],triggerTimeIndex,adccount,adccountt;
unsigned char CA_arr[28] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90,
    0xBF, 0xC7, 0xC1, 0x8C, 0xC6, 0x83, 0xE3, 0xAF, 0x89, 0x86, 0x8E, 0x87, 
    0xA3, 0xA1, 0x91, 0xAB, 0x88, 0xFF}; //0-9,-10,L11,U12,P13,C14,b15,u16,
    //r17,H18,E19,F20,t21,o22,d23,y24,n25,A26,null27

unsigned char off12,off34,off56;
int alphaCount1,alphaCount2,alphaCount3,DegreeController;
unsigned int ch0, ch1, ch2, avg_ch0, avg_ch1, avg_ch2, volt_ch00,cur_ch11;
unsigned long volt_ch0,cur_ch1;
void ProgInit() 
{
    //general i/o config
    OSCCON=0x75;//8mhz
    TRISA = 0b00000111;
    TRISB = 0b00000111;
    TRISC = 0b00000000;
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    //common config
    ANSEL = 0x00; //Disable analog I/O ,IF ANY ANALOG PIN REQUIRED ENABLE THAT
    ANSELH = 0x00; //Disable analog I/O high bits ,IF ANY ANALOG PIN REQUIRED ENABLE THAT 
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;


}
void Pwm_50K_ccp1()
{
    PR2 = 0b00000100 ;
    T2CON = 0b00000101 ;
    CCPR1L = 0b00000010 ;
    CCP1CON = 0b00011100 ;
}
void Pwm_50K_ccp2()
{
    PR2 = 0b00000100 ;
    T2CON = 0b00000101 ;
    CCPR2L = 0b00000010 ;
    CCP2CON = 0b00011100 ;
}
void IOCInit()
{
    //soft Uart config
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    
    OPTION_REGbits.nRBPU = 1;
    WPUBbits.WPUB0 = 0;
    WPUBbits.WPUB1 = 0;
    WPUBbits.WPUB2 = 0;
    
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB1 = 1;
    IOCBbits.IOCB2 = 1;
    INTCONbits.RBIF = 0;
    INTCONbits.RBIE = 1;
}
void TimerINTinit()
{
    //timer0 interupt config
    OPTION_REG = 0x00;//interrupt at every .5 ms
    INTCONbits.T0IF = 0;
    INTCONbits.T0IE = 1;   
}
void SevenSeg(int d_h,int d_t,int d_u)
 {
     digitcount++;
      switch(digitcount)
       {
           case 1:comh=1;comt=0;comu=0;
                  PORTB= CA_arr[d_h];
                  if(ph==1){PORTBbits.RB7=0;}
                  break;
           
           case 2:comh=0;comt=1;comu=0;
                  PORTB= CA_arr[d_t];
                  if(pt==1){PORTBbits.RB7=0;}
                  break;
           
           case 3:comh=0;comt=0;comu=1;
                  PORTB= CA_arr[d_u];
                  digitcount=0;
                  if(pu==1){PORTBbits.RB7=0;}
                  break;

       }
 }
void dispConFun(int DH, int DT, int DU)
{
    dh=DH;
    dt=DT;
    du=DU;
}

void ADC_init() {

    ANSELbits.ANS0 = 1; //pin2
    ANSELbits.ANS1 = 1; //pin3
    ANSELbits.ANS2 = 1; //pin4

    ADCON1 = 0x80; //ADC INIT
    ADCON0 = 0x40;


}

unsigned int ADC(int ch_no) 
{
    unsigned int val;
    ADCON0 = (ch_no == 0) ? 0x41 :
            (ch_no == 1) ? 0x45 :
            (ch_no == 2) ? 0x49 : 0x40;

    __delay_us(20);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO_nDONE == 1);
    val = ADRESH * 255 + ADRESL;
    return val;

}


char eepread(int loc)
{
EEADR=loc;
EECON1bits.RD=1;
while(EECON1bits.RD==1);
__delay_ms(5);
return EEDATA;
}

void eepwrite(char d,int loc)
{
EEADR=loc;
EEDATA=d;
EECON1bits.EEPGD = 0;
EECON1bits.WREN=1;
INTCONbits.GIE = 0;
EECON2=0x55;
EECON2=0xAA;
EECON1bits.WR=1;
while(EECON1bits.WR==1);
INTCONbits.GIE = 1;
__delay_ms(5);
}

void SCRfiringAndSequenceAndTimeMange()
{
    
    
}
void interrupt Isr()
{
     if(INTCONbits.RBIF==1)
        {
            INTCONbits.GIE=0;
            INTCONbits.RBIE=0;
            if(rzcd==1 && yzcd==0 && bzcd==1)
            {
//                thy3GF=0;
//                thy4GF=0;
                triggerCon[triggerConIndex]='R';
                triggerConIndex++;
                INTCONbits.T0IF=1;
            }
            else if(rzcd==0 && yzcd==1 && bzcd==0)
            {
               thy1G=0;
               thy2G=0;
            }
            else if(rzcd==1 && yzcd==1 && bzcd==0)
            {
//                thy5GF=0;
//                thy6GF=0;
                triggerCon[triggerConIndex]='Y';
                triggerConIndex++;
                INTCONbits.T0IF=1;
            }
            else if(rzcd==0 && yzcd==0 && bzcd==1)
            {
                thy3G=0;
                thy4G=0;
            }
            else if(rzcd==0 && yzcd==1 && bzcd==1)
            {
//                thy1GF=0;
//                thy2GF=0;
                triggerCon[triggerConIndex]='B';
                triggerConIndex++;
                INTCONbits.T0IF=1;
            }
            else if(rzcd==1 && yzcd==0 && bzcd==0)
            {
                thy5G=0;
                thy6G=0;
            }

            INTCONbits.GIE=1;
            INTCONbits.RBIE=1;
            INTCONbits.RBIF=0;
        }
      
     if(INTCONbits.T0IF==1)
     {
         if(triggerCon[0] != '\0')triggerTimeCon[0]++;
         if(triggerCon[1] != '\0')triggerTimeCon[1]++;
        
         if(triggerTimeCon[0]>=DegreeController)
            {
                int i=0;
                switch(triggerCon[0])
                {
                    case 'R':thy1GF=1;thy2GF=1;break;
                    case 'Y':thy3GF=1;thy4GF=1;break;
                    case 'B':thy5GF=1;thy6GF=1;break;
                    default:break;  
                }

                for(i=0;triggerCon[i+1]!='\0';i++)
                {
                    triggerCon[i]=triggerCon[i+1];
                    triggerTimeCon[i]=triggerTimeCon[i+1];

                }
                    triggerTimeCon[i]=0;
                    triggerCon[i]='\0';
                    if(triggerConIndex>0)triggerConIndex--;   
            }
         
         thy1G=(thy1GF)? ~thy1G:0;
         thy2G=(thy2GF)? ~thy2G:0;
         thy3G=(thy3GF)? ~thy3G:0;
         thy4G=(thy4GF)? ~thy4G:0;
         thy5G=(thy5GF)? ~thy5G:0;
         thy6G=(thy6GF)? ~thy6G:0;
         
         
         TMR0=205;
         INTCONbits.T0IF=0; 
         
     }
}
void main(void) 
{
    ProgInit();
    __delay_ms(5000);
    ADC_init();
    IOCInit();
    TimerINTinit();
    while(1)
    {
        if (adccount < 15)
        {
            ch0 = ch0 + ADC(0);
            ch1 = ch1 + ADC(1);
            ch2 = ch2 + ADC(2);
            adccount++;
            if(adccount >= 15)
            {
               avg_ch2 = ch2/adccount;
               DegreeController = 195 - (avg_ch2*19)/100;
               adccount = 0;
            }
 
        }
    }
    return;
}
