#include "mbed.h"

BufferedSerial uart0(P0_25, P0_26,115200);  //TX, RX
I2C i2c(P0_27,P0_28);         //SDA,SCL dds dac
SPI dds(P0_9,P0_8,P0_7);    //dds
DigitalOut le1(P0_6);       //dds1
DigitalOut le2(P0_5);       //dds2
DigitalOut ps(P0_4);       //dds2 phase select
SPI adc(P0_18,P0_17,P0_15);    //adc
DigitalOut cs(P0_20);       //adc cs
DigitalIn drdy(P0_19);   //adc drdy

DigitalOut i0(P0_0);    //100 lo on
DigitalOut i1(P0_1);    //1k
DigitalOut i2(P0_10);   //10k
DigitalOut i3(P0_11);   //100k

DigitalOut v0(P1_29);   //0=x1, 1=x2, 2=x4, 3=x8
DigitalOut v1(P1_28);

//uart control
const uint8_t buf_size=17;
char read_buf[buf_size];    //uart read buf
void buf_read(uint8_t num); //uart read func.
void buf2val();             //buf to vals change func. return to 'freq' and 'ampl' global var 
void val_send(float val);  //uart send func.

//DDS control
#define res_inv 4           //res=67108864/2^28
void waveset(uint32_t freq,uint16_t ampl);    //waveset func.
uint32_t freq;              //Hz
uint16_t ampl;          //deg. loaded mV

//DAC control
#define dac_fs 3300     //DAC full scale Vout
#define dac_res 4096    //dac resolution 2^12
#define g 4             //driver amp gain
const uint8_t dac_addr=0xc0;

//adc control
const uint8_t rst=0b0110;
const uint8_t wreg=0b0100;
const uint8_t start=0b1000;
void drdy_wait();
int16_t adc_read(int8_t ch);

//calc.
uint8_t integ,i,j;
int16_t im[2],vm[2];
float im_f[2],vm_f[2],den,z_re,z_im,vg,ig;
uint8_t i_gain,v_gain;
void ig_set();      //I gain set func.
void vg_set();      //V gain set func.

int main(){
    le1=1;
    le2=1;
    ps=0;
    cs=1;
    dds.format(16,2);   //spi mode setting. 2byte(16bit) transfer, mode 2
    adc.format(8,1);

    i0=0;
    i1=1;
    i2=1;
    i3=1;
    v0=0;
    v1=0;

    //adc init
    cs=0;
    adc.write(rst);
    cs=1;
    thread_sleep_for(10);
    cs=0;
    adc.write((wreg<<4)+(0x01<<2)+0);    //write addr 0x01, 1byte
    adc.write((0b010<<5)+(0b10<<3)+(0b1<<2));       //180sps, turbo mode, cc mode
    cs=1;

    while (true){
        buf_read(buf_size); //uart buf read
        buf2val();
        waveset(freq,ampl); //dds set
        ig_set();   //I gain set
        vg_set();   //V gain set
        for(i=0;i<2;++i){   //meas
            im_f[i]=0;
            vm_f[i]=0;
            ps=i;
            thread_sleep_for(10);
            for(j=0;j<integ;++j){
                im[i]=adc_read(0);
                im_f[i]=im_f[i]+(float)im[i]*-1;
            }
            for(j=0;j<integ;++j){
                vm[i]=adc_read(1);
                vm_f[i]=vm_f[i]+(float)vm[i];
            }
            im_f[i]=im_f[i]/integ/ig;
            vm_f[i]=vm_f[i]/integ/vg;
        }
        
        //calc. and send
        den=im_f[0]*im_f[0]+im_f[1]*im_f[1];    //Re(im)^2+Im(im)^2
        z_re=(vm_f[0]*im_f[0]+vm_f[1]*im_f[1])/den;
        z_im=(vm_f[1]*im_f[0]-vm_f[0]*im_f[1])/den;
        val_send(z_re);
        val_send(z_im);
        
        /*//debug
        int z_ree,z_imm;
        z_ree=(int)z_re;
        z_imm=(int)z_im;
        printf("Freq=%d, Ampl=%d, Integ=%d, I_gain=%d, V_gain=%d, Re(Z)=%d, Im(Z)=%d\n\r",freq, ampl, integ, i_gain, v_gain, z_ree,z_imm);
        */
    }
}

//uart char number read func.
void buf_read(uint8_t num){
    char local_buf[1];
    uint8_t i;
    for (i=0;i<num;++i){
        uart0.read(local_buf,1);
        read_buf[i]=local_buf[0];
    }
}

//buf to val change func.
void buf2val(){
    uint8_t i,j;
    uint32_t pow10;
    freq=0;
    ampl=0;
    integ=0;
    i_gain=0;
    v_gain=0;
    for(i=0;i<8;++i){
        pow10=1;
        for(j=0;j<7-i;++j){
            pow10=10*pow10;
        }
        freq=freq+(read_buf[i]-48)*pow10;
    }
    for(i=0;i<4;++i){
        pow10=1;
        for(j=0;j<3-i;++j){
            pow10=10*pow10;
        }
        ampl=ampl+(read_buf[i+8]-48)*pow10;
    }
    for(i=0;i<3;++i){
        pow10=1;
        for(j=0;j<2-i;++j){
            pow10=10*pow10;
        }
        integ=integ+(read_buf[i+12]-48)*pow10;
    }
    i_gain=read_buf[15]-48;
    v_gain=read_buf[16]-48;
}

//uart send func.
void val_send(float val){
    char local_buf[1];
    char data[7];
    uint8_t i;
    uint64_t integer;
    if(val<0){
        val=abs(val);
        local_buf[0]=45;
        uart0.write(local_buf,1);   //send minus
    }else{
        val=val;
        local_buf[0]=43;
        uart0.write(local_buf,1);   //send plus
    }
    integer=(uint64_t)(val*1000);
    data[6]=0x30+(integer/1000)%10;        //1
    data[5]=0x30+(integer/10000)%10;       //10
    data[4]=0x30+(integer/100000)%10;      //100
    data[3]=0x30+(integer/1000000)%10;     //1000
    data[2]=0x30+(integer/10000000)%10;    //10000
    data[1]=0x30+(integer/100000000)%10;   //100000
    data[0]=0x30+(integer/1000000000)%10;  //1000000
    for(i=0;i<7;++i){
        local_buf[0]=data[i];
        uart0.write(local_buf,1);
    }

    local_buf[0]=46;
    uart0.write(local_buf,1);   //send '.'
    
    data[2]=0x30+integer%10;            //0.001
    data[1]=0x30+(integer/10)%10;       //0.01
    data[0]=0x30+(integer/100)%10;      //0.1
    for(i=0;i<3;++i){
        local_buf[0]=data[i];
        uart0.write(local_buf,1);
    }
}

//wave set func.
void waveset(uint32_t freq, uint16_t ampl){
    uint16_t buf;   //spi send buf
    char set[2];    //i2c send buf
    uint16_t pha=80; //for adjust
    if(freq>30000000)freq=30000000;
    if(ampl>2400)ampl=2400;

    //dds1 stimulus
    le1=0;
    dds.write(0x2100);
    buf=((res_inv*freq)&0x3FFF)+0x4000;
    dds.write(buf);
    buf=((res_inv*freq)>>14)+0x4000;
    dds.write(buf);
    buf=(4096*pha/360)+0xC000;  //pha reg 0 = 0 deg, adjusted
    dds.write(buf);
    le1=1;
    
    //dds2 LO
    le2=0;
    dds.write(0x2100);
    buf=((res_inv*freq)&0x3FFF)+0x4000;
    dds.write(buf);
    buf=((res_inv*freq)>>14)+0x4000;
    dds.write(buf);
    buf=0+0xC000;  //pha reg 0 = 0 deg
    dds.write(buf);
    buf=(4096*90/360)+0xE000; //pha reg 0 = 90 deg
    dds.write(buf);
    le2=1;

    le1=0;
    le2=0;
    dds.write(0x2000+0x200);      //accum. reset, pi/sw=1, pin sel mode
    le1=1;
    le2=1;

    buf=((1200-2*ampl/g)*dac_res/dac_fs);    //(1/res)*(1200/3)*(3-ampl*2/(200*g))
    set[0]=buf>>8;
    set[1]=buf&0xff;
    if(i==1){
        i2c.write(dac_addr,set,2);  //ch1 dac
    }else{
        i2c.write(dac_addr+0x2,set,2);  //ch2 dac
    }
}

void drdy_wait(){
    while(true){
        if(drdy==0) break;
    }
}

int16_t adc_read(int8_t ch){
    uint8_t buf[2];     //spi receive buf
    cs=0;
    adc.write((wreg<<4));       //write addr 0x00, 1byte
    if(ch==0)adc.write((0b0000<<4)+1);   //ch0 ch1 bipolar mux, pga disable h81
    if(ch==1)adc.write((0b0101<<4)+1);   //ch2 ch3 bipolar mux, pga disable h81
    cs=1;

    drdy_wait();
    cs=0;
    buf[1]=adc.write(0x00);
    buf[0]=adc.write(0x00);
    cs=1;
    return(buf[1]<<8)+buf[0];
}

//I gain set func.
void ig_set(){
    switch (i_gain) {
        case 0:
            i0=0;
            i1=1;
            i2=1;
            i3=1;
            ig=100;
        break;
        case 1:
            i0=1;
            i1=0;
            i2=1;
            i3=1;
            ig=1000;
        break;
        case 2:
            i0=1;
            i1=1;
            i2=0;
            i3=1;
            ig=10000;
        break;
        case 3:
            i0=1;
            i1=1;
            i2=1;
            i3=0;
            ig=100000;
        break;
    }
}

//V gain set func.
void vg_set(){
    switch (v_gain) {
        case 0:
            v0=0;
            v1=0;
            vg=1;
        break;
        case 1:
            v0=1;
            v1=0;
            vg=2;
        break;
        case 2:
            v0=0;
            v1=1;
            vg=4;
        break;
        case 3:
            v0=1;
            v1=1;
            vg=8;
        break;
    }
}
