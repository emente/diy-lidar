/*
uses 20mhz atmega168 5v with external 20mhz crystal for avr+sensor

cables from slip ring:
 red +5v
 blk gnd
 grn rst  avr reset
 wht rxd  serial
 blu txd  serial
 yel 5    motor pwm, use some mosfet to drive motor
 
cables from pixart sensor:
 red +3.3v
 blk gnd
 yel sda
 wht scl
 grn rst
 blu clk 
 
pixart sensor:
 ....... lens
 ... +----------+
 +--+----------+--+
 ... !  7 5 3 1 ! bottom
 ... ! 8 6 4 2  !
 ... +----------+
 1 +3.3v
 2 gnd
 3 gnd
 4 nc
 5 scl
 6 sda
 7 clk (20-25 mhz)
 8 rst  
 */

///////////////////////////////////////////////////

#include <digitalWriteFast.h> 
#include <Wire.h>
#include <EEPROM.h>

#define P_ROTSENSOR 2
#define P_LASER1 6
#define P_MOTOR 5
#define P_LED 13

#define IRslaveAddress 0xB0 >> 1

const char hexchars[17]="0123456789ABCDEF";

uint8_t motorPwm = 60;
volatile boolean scanFlag = false;
volatile boolean loopFlag = false;
volatile boolean rotFlag = false;
volatile uint16_t loopCnt=0;
uint16_t oldLoopCnt=0;

char serbuf[16];
uint8_t serpos = 0;

struct TBlob
{
    uint16_t X;
    uint16_t Y;
    uint8_t Size;
};

TBlob blobs[5]; //5th is used for swap while sorting

//////////////////////////////////////////////////////////////////

void initPixart() {
    int p0,p1,p2,p3;

    // highest
    p0=0x72; 
    p1=0x20; 
    p2=0x1F; 
    p3=0x03;
    //p0=0xC8; p1=0x36; p2=0x35; p3=0x03;
    //p0=0xAA; p1=0x64; p2=0x63; p3=0x03;
    //p0=0x96; p1=0xB4; p2=0xB3; p3=0x04;
    /*
    //lowest
     p0=0x96; 
     p1=0xFE; 
     p2=0xFE; 
     p3=0x05;
     */
    Wire.begin();

    Wire.beginTransmission(IRslaveAddress);
    Wire.write( 0x30 ); 
    Wire.write( 0x01 );
    Wire.endTransmission();

    Wire.beginTransmission(IRslaveAddress);
    Wire.write( 0x00 );
    Wire.write( 0x02 ); 
    Wire.write( 0x00 ); 
    Wire.write( 0x00 ); 
    Wire.write( 0x71 );
    Wire.write( 0x01 ); 
    Wire.write( 0x00 ); 
    Wire.write( p0 );
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(IRslaveAddress);
    Wire.write( 0x07 ); 
    Wire.write( 0x00 ); 
    Wire.write( p1 );
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(IRslaveAddress);
    Wire.write( 0x1A ); 
    Wire.write( p2 ); 
    Wire.write( p3 );
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(IRslaveAddress);
    Wire.write( 0x33 ); 
    Wire.write( 0x03 );
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(IRslaveAddress);
    Wire.write( 0x30 ); 
    Wire.write( 0x08 );
    Wire.endTransmission();
    delay(500);
}

void init500Hz() {
    noInterrupts();

    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR1A = 59250;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    interrupts();        
}

void motorPower(boolean on) {
    if (on) {
        analogWrite(P_MOTOR,motorPwm);
    } 
    else {
        digitalWrite(P_MOTOR,LOW);   
    }
}

void scanPower(boolean on) {
    scanFlag=on;
    digitalWrite(P_LED,LOW);
    digitalWrite(P_LASER1,on?HIGH:LOW);
    digitalWrite(12,on?HIGH:LOW);
    digitalWrite(11,on?HIGH:LOW);
    digitalWrite(10,on?HIGH:LOW);
}

void setup() {
    Serial.begin(115200);

    pinMode(P_ROTSENSOR,INPUT);
    pinMode(P_LASER1,OUTPUT);
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(12,OUTPUT);
    pinMode(P_MOTOR,OUTPUT);
    pinMode(P_LED,OUTPUT);

    //divisor 0 for D5/D6
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of

    int i=EEPROM.read(0);
    if (i!=255) {
        motorPwm=i;
    }

    motorPower(false);
    scanPower(false);

    initPixart();
    attachInterrupt(0,fullRotation,RISING);
    init500Hz();

    Serial.println("");
}

//////////////////////////////////////////////////////////////////

void serialLoop() {
    while (Serial.available()) {
        digitalWrite(13,digitalRead(13)==LOW?HIGH:LOW);
        char r=Serial.read();

        switch (r) {

        case'1':
            scanPower(true);
            motorPower(true);   
            break;

        case '0':
            scanPower(false);
            motorPower(false);   
            break;

        case'?':
            Serial.println("!");   
            break;

        case 'm':
            motorPower(false);   
            break;
            
        case 'M':
            motorPower(true);   
            break;

        case 'l':
            scanPower(false);
            break;
            
        case '-':
            motorPwm--;
            motorPower(true);
            Serial.print('w');
            Serial.println(motorPwm);
            EEPROM.write(0,motorPwm);
            break;

        case '+':
            motorPwm++;
            motorPower(true);
            Serial.print('w');
            Serial.println(motorPwm);
            EEPROM.write(0,motorPwm);
            break;

        case 'a':
            digitalWrite(P_LASER1,HIGH);
            digitalWrite(12,LOW);
            digitalWrite(11,LOW);
            digitalWrite(10,LOW);
            break;

        case 'b':
            digitalWrite(P_LASER1,LOW);
            digitalWrite(12,HIGH);
            digitalWrite(11,HIGH);
            digitalWrite(10,HIGH);
            break;

        case 'c':
            digitalWrite(P_LASER1,HIGH);
            digitalWrite(12,HIGH);
            digitalWrite(11,HIGH);
            digitalWrite(10,HIGH);
            break;   
        }
    }
}

void fullRotation() {
    loopCnt=0;
    rotFlag=true;
}

ISR( TIMER1_COMPA_vect ) {
    if (!scanFlag) {
        return;   
    }

    loopFlag=!loopFlag;
    if (loopFlag) {
        loopCnt++;
        digitalWriteFast2(P_LED,loopCnt & 4?HIGH:LOW);
    }
}

float toRad(float deg) {
    return deg*(3.14159265/180); 
}

uint8_t getDepth(uint16_t x) {
    const uint16_t pixels = 1023-x;    
    const float width = 1024;
    const float pixelsPerDeg=5;

    float A=toRad(85);
    float B;
    float C;

    float a;
    float c=7;
    if (pixels > width/2) {
        B=toRad(90+((pixels-width / 2) / pixelsPerDeg));
    } 
    else {
        B=toRad(90-((width/2 - pixels) / pixelsPerDeg));   
    }
    C= toRad(180)-(B+A);
    a=(c*sin(A))/sin(C);

    return a;    
}


void loop() {
    uint8_t i,s,j;
    byte data_buf[16];

    serialLoop();

    if (scanFlag && loopCnt!=oldLoopCnt) {
        oldLoopCnt=loopCnt;   

        if (rotFlag) {
             rotFlag=false;
             Serial.println("l-----------");
        }

        Wire.beginTransmission(IRslaveAddress);
        Wire.write(0x36);
        Wire.endTransmission();

        for (i=0;i<16;i++)        {
            data_buf[i]=0;
        }

        i=0;
        Wire.requestFrom(IRslaveAddress, 16);
        while(Wire.available() && i < 16)        {
            data_buf[i] = Wire.read();
            i++;
        }

        for (i=0;i<4;i++) {
            uint8_t off=(i*3)+1;

            blobs[i].Y = data_buf[off];
            blobs[i].X = data_buf[off+1];
            s   = data_buf[off+2];
            blobs[i].Y += (s & 0x30) <<4;
            blobs[i].X += (s & 0xC0) <<2;
            blobs[i].Size = (s & 0x0F);
        }

        for (i=0;i<4;i++) {
            if (blobs[i].X==1023 && blobs[i].Y==1023 && blobs[i].Size == 15){
                blobs[i].Size=0;
            }
            blobs[i].X = 1023-blobs[i].X;
            blobs[i].Y = 1023-blobs[i].Y;
        }

        //sort by Y
        for (i=0;i<4;i++) {
            for (j=0;j<4;j++) {
                if (blobs[i].Y > blobs[i].Y) {
                    //swap
                    blobs[4].X = blobs[i].X;
                    blobs[4].Y = blobs[i].Y;
                    blobs[4].Size = blobs[i].Size;

                    blobs[i].X = blobs[j].X;
                    blobs[i].Y = blobs[j].Y;
                    blobs[i].Size = blobs[j].Size;

                    blobs[j].X = blobs[4].X;
                    blobs[j].Y = blobs[4].Y;
                    blobs[j].Size = blobs[4].Size;
                }
            }
        }

        // distance laser<>sensor 70mm, 5°
        for (i=0;i<4;i++) {
            //blobs[i].X = getDepth(blobs[i].X);  
        }

        Serial.write(76);
        for (i=0;i<4;i++) {
            Serial.write(hexchars[(blobs[i].X >> 8)&0x0F]);
            Serial.write(hexchars[(blobs[i].X >> 4)&0x0F]);
            Serial.write(hexchars[blobs[i].X&0x0F]);

            Serial.write(hexchars[(blobs[i].Y >> 8)&0x0F]);
            Serial.write(hexchars[(blobs[i].Y >> 4)&0x0F]);
            Serial.write(hexchars[blobs[i].Y&0x0F]);

            Serial.write(hexchars[blobs[i].Size]);
            Serial.write(32);            
        }
        Serial.write(hexchars[(loopCnt >> 4)&0x0F]);
        Serial.write(hexchars[loopCnt&0x0F]);
        Serial.write(10);
        Serial.flush();
    }
    
    if (!scanFlag) {
         if (loopCnt>100) {
              loopCnt=0;
         }       
    }
}
