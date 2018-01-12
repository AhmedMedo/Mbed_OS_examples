#include "mbed.h"
#include "rtos.h"
#include "SWO.h"
#include "config.h"
#include "filesystem/bd/BlockDevice.h"
#include "filesystem/bd/HeapBlockDevice.h"



SWO_Channel swo("channel");

#if RTOS_EXAMPLE
Serial pc(SERIAL_TX, SERIAL_RX);


void print_char(char c = '*')
{
    swo.printf("%c", c);
    fflush(stdout);
}

DigitalOut led1(LED1);

void print_thread(void const *argument)
{
    while (true) {
        Thread::wait(1000);
        print_char();
    }
}

int main()
{
    swo.printf("\n\n*** RTOS basic example ***\n");
    Thread thread(print_thread, NULL, osPriorityNormal, DEFAULT_STACK_SIZE);
    while (true) {
        led1 = !led1;
        Thread::wait(500);
    }
}
#endif

#if TICKER_EXAMPLE
class Flipper {
public:
    Flipper(PinName pin) : _pin(pin) {
        _pin = 0;
    }
    void flip() {
        _pin = !_pin;
    }
private:
    DigitalOut _pin;
};
 
DigitalOut led1(LED1);
Flipper f(LED2);
Ticker t;
 
int main() {
    // the address of the object, member function, and interval
    t.attach(callback(&f, &Flipper::flip), 2.0); 
 
    // spin in a main loop. flipper will interrupt it to call flip
    while(1) {
        led1 = !led1;
        wait(0.2);
    }
}

#endif
#if TIME_EXAMPLE
int main() {
    set_time(1256729737);  // Set RTC time to Wed, 28 Oct 2009 11:35:37

    while (true) {
        time_t seconds = time(NULL);
        
        swo.printf("Time as seconds since January 1, 1970 = %d\n", seconds);
        
        swo.printf("Time as a basic string = %s", ctime(&seconds));

        char buffer[32];
        strftime(buffer, 32, "%I:%M %p\n", localtime(&seconds));
        swo.printf("Time as a custom formatted string = %s", buffer);
        
        wait(1);
    }
}
#endif
#if TIMEOUT_EXAMPLE
// A class for flip()-ing a DigitalOut 
class Flipper {
public:
    Flipper(PinName pin) : _pin(pin) {
        _pin = 0;
    }
    void flip() {
        _pin = !_pin;
    }
private:
    DigitalOut _pin;
};

DigitalOut led1(LED1);
Flipper f(LED2);
Timeout t;

int main() {
    // the address of the object, member function, and interval
    t.attach(callback(&f, &Flipper::flip), 2.0); 

    // spin in a main loop. flipper will interrupt it to call flip
    while(1) {
        led1 = !led1;
        wait(0.2);
    }
}


#endif

#if TIMER_EXAMPLE
Timer t;
 
int main() {
    t.start();
    printf("Hello World!\n");
    t.stop();
    printf("The time taken was %f seconds\n", t.read());
}

#endif



#if DIGITAL_IN_EXAMPLE

DigitalIn  mypin(SW2); // change this to the button on your board
DigitalOut myled(LED1);

int main()
{
    // check mypin object is initialized and connected to a pin
    if(mypin.is_connected()) {
        printf("mypin is connected and initialized! \n\r");
    }
    
    // Optional: set mode as PullUp/PullDown/PullNone/OpenDrain
    mypin.mode(PullNone); 
    
    // press the button and see the console / led change
    while(1) {
        printf("mypin has value : %d \n\r", mypin.read());
        myled = mypin; // toggle led based on value of button
        wait(0.25);
    }
}



#endif
#if DIGITAL_OUT_EXAMPLE

DigitalOut myled(LED1);

int main() {
       // check that myled object is initialized and connected to a pin
    if(myled.is_connected()) {
        swo.printf("myled is initialized and connected!\n\r");
    }

    // Blink LED
    while(1) {
        myled = 1;          // set LED1 pin to high
        swo.printf("\n\r myled = %d", (uint8_t)myled );
        wait(0.5);

        myled.write(0);     // set LED1 pin to low
        swo.printf("\n\r myled = %d",myled.read() );
        wait(0.5);
    }

}


#endif

#if DIGITAL_IN_OUT_EXAMPLE
DigitalInOut mypin(LED1);

int main()
{
    // check that mypin object is initialized and connected to a pin
    if(mypin.is_connected()) {
        printf("mypin is initialized and connected!\n\r");
    }

    // Optional: set mode as PullUp/PullDown/PullNone/OpenDrain
    mypin.mode(PullNone);

    while(1) {
        // write to pin as output
        mypin.output();
        mypin = !mypin; // toggle output
        wait(0.5);

        // read from pin as input
        mypin.input();
        printf("mypin.read() = %d \n\r",mypin.read());
        wait(0.5);
    }
}
#endif
#if ANALOG_IN_EXAMPLE
// Initialize a pins to perform analog input and digital output fucntions
AnalogIn   ain(A0);
DigitalOut dout(LED1);

int main(void)
{
    while (1) {
        // test the voltage on the initialized analog pin
        //  and if greater than 0.3 * VCC set the digital pin
        //  to a logic 1 otherwise a logic 0
        if(ain > 0.3f) {
            dout = 1;
        } else {
            dout = 0;
        }
        
        // print the percentage and 16 bit normalized values
        swo.printf("percentage: %3.3f%%\n", ain.read()*100.0f);
        swo.printf("normalized: 0x%04X \n", ain.read_u16());
        wait(0.2f);
    }
}


#endif

#if BUS_IN_EXAMPLE
BusIn nibble(D0, D1, D2, D3); // Change these pins to buttons on your board.

int main() {
    
    // Optional: set mode as PullUp/PullDown/PullNone/OpenDrain
    nibble.mode(PullNone); 
    
    while(1) {
        // check bits set in nibble
        switch(nibble & nibble.mask()) { // read the bus and mask out bits not being used
            case 0x0: swo.printf("0b0000, D3,D2,D1,D0 are low  \n\r");break;
            case 0x1: swo.printf("0b0001,          D0  is high \n\r");break;
            case 0x2: swo.printf("0b0010,       D1     is high \n\r");break; 
            case 0x3: swo.printf("0b0011,       D1,D0 are high \n\r");break;
            case 0x4: swo.printf("0b0100,    D2        is high \n\r");break;
            case 0x5: swo.printf("0b0101,    D2,  ,D0 are high \n\r");break;
            case 0x6: swo.printf("0b0110,    D2,D1    are high \n\r");break;
            case 0x7: swo.printf("0b0111,    D2,D1,D0 are high \n\r");break;
            case 0x8: swo.printf("0b1000, D3           is high \n\r");break; 
            // ...
            case 0xF: swo.printf("0b1111, D3,D2,D1,D0 are high \n\r");break;
        }
        wait(1);
    }
}

#endif

#if BUS_OUT_EXAMPLE
BusOut myleds(LED1, LED2, LED3, LED4);
 
int main() {
    while(1) {
        for(int i=0; i<16; i++) {
            myleds = i;
            wait(0.25);
        }
    }
}

#endif
#if BUS_IN_OUT_EXAMPLE
int main() {
    while(1) {
        pins.output();
        pins = 0x3;
        wait(1);
        pins.input();
        wait(1);
        if(pins == 0x6) {
            printf("Hello!\n");
        }
    }
}


#endif

#if PORT_IN_EXAMPLE
PortIn     p(Port2, 0x0000003F);   // p21-p26
DigitalOut ind(LED4);
 
int main() {
    while(1) {
        int pins = p.read();
        if(pins) {
            ind = 1;
        } else {
            ind = 0;
        }
    }
}

#endif

#if PORT_OUT_EXAMPLE
PwmOut led(LED1);

int main() {
    // specify period first
    led.period(4.0f);      // 4 second period
    led.write(0.50f);      // 50% duty cycle, relative to period
    //led = 0.5f;          // shorthand for led.write()
    //led.pulsewidth(2);   // alternative to led.write, set duty cycle time in seconds
    while(1);
}


#endif
#if PORT_IN_OUT_EXAMPLE
// LED1 = P1.18  LED2 = P1.20  LED3 = P1.21  LED4 = P1.23
#define LED_MASK 0x00B40000
 
PortInOut ledport(Port1, LED_MASK);
 
int main() {
    int v = ledport;
    ledport.output();
    while(1) {
        ledport = LED_MASK;
        wait(0.5);
        ledport = 0;
        wait(1);
    }
}
#endif

#if PWM_EXAMPLE

PwmOut led(LED2);

int main() {
    // specify period first, then everything else
    led.period(4.0f);  // 4 second period
    led.pulsewidth(2); // 2 second pulse (on)
    while(1);          // led flashing
}

#endif

#if INTERRUPT_IN_EXAMPLE

InterruptIn button(SW2);
DigitalOut led(LED1);
DigitalOut flash(LED4);
 
void flip() {
    led = !led;
}
 
int main() {
    button.rise(&flip);  // attach the address of the flip function to the rising edge
    while(1) {           // wait around, interrupts will interrupt this!
        flash = !flash;
        wait(0.25);
    }
}
#endif

#if SERIAL_EXAMPLE

Serial device(USBTX, USBRX);  // tx, rx

int main() {
    device.baud(19200);
    device.printf("Hello World\n");
}


#endif

#if SPI_MASTER_EXAMPLE
SPI spi(D11, D12, D13); // mosi, miso, sclk
DigitalOut cs(D0);
 
int main() {
    // Chip must be deselected
    cs = 1;

    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8,3);
    spi.frequency(1000000);
 
    // Select the device by seting chip select low
    cs = 0;
 
    // Send 0x8f, the command to read the WHOAMI register
    spi.write(0x8F);
 
    // Send a dummy byte to receive the contents of the WHOAMI register
    int whoami = spi.write(0x00);
    swo.printf("WHOAMI register = 0x%X\n", whoami);
 
    // Deselect the device
    cs = 1;
}

#endif

#if SPI_SLAVE_EXAMPLE

SPISlave device(p5, p6, p7, p8); // mosi, miso, sclk, ssel

int main() {
   device.reply(0x00);              // Prime SPI with first reply
   while(1) {
       if(device.receive()) {
           int v = device.read();   // Read byte from master
           v = (v + 1) % 0x100;     // Add one to it, modulo 256
           device.reply(v);         // Make this the next reply
       }
   }
}


#endif
#if I2C_MASTER_EXAMPLE
// Read temperature from LM75BD

I2C i2c(I2C_SDA , I2C_SCL ); 

const int addr7bit = 0x48;      // 7 bit I2C address
const int addr8bit = 0x48 << 1; // 8bit I2C address, 0x90

int main() {
    char cmd[2];
    while (1) {
        cmd[0] = 0x01;
        cmd[1] = 0x00;
        i2c.write(addr8bit, cmd, 2);
 
        wait(0.5);
 
        cmd[0] = 0x00;
        i2c.write(addr8bit, cmd, 1);
        i2c.read( addr8bit, cmd, 2);
 
        float tmp = (float((cmd[0]<<8)|cmd[1]) / 256.0);
        printf("Temp = %.2f\n", tmp);
    }
}
#endif

#if I2C_SLAVE_EXAMPLE

I2CSlave slave(p9, p10);
int main() {
   char buf[10];
   char msg[] = "Slave!";

   slave.address(0xA0);
   while (1) {
       int i = slave.receive();
       switch (i) {
           case I2CSlave::ReadAddressed:
               slave.write(msg, strlen(msg) + 1); // Includes null char
               break;
           case I2CSlave::WriteGeneral:
               slave.read(buf, 10);
               printf("Read G: %s\n", buf);
               break;
           case I2CSlave::WriteAddressed:
               slave.read(buf, 10);
               printf("Read A: %s\n", buf);
               break;
       }
       for(int i = 0; i < 10; i++) buf[i] = 0;    // Clear buffer
   }
}

#endif
#if CAN_EXAMPLE

Ticker ticker;
DigitalOut led1(LED1);
DigitalOut led2(LED2);
CAN can1(MBED_CONF_APP_CAN1_RD, MBED_CONF_APP_CAN1_TD);
CAN can2(MBED_CONF_APP_CAN2_RD, MBED_CONF_APP_CAN2_TD);
char counter = 0;

void send() {
    printf("send()\n");
    if(can1.write(CANMessage(1337, &counter, 1))) {
        printf("wloop()\n");
        counter++;
        printf("Message sent: %d\n", counter);
    } 
    led1 = !led1;
}

int main() {
    printf("main()\n");
    ticker.attach(&send, 1);
    CANMessage msg;
    while(1) {
        printf("loop()\n");
        if(can2.read(msg)) {
            printf("Message received: %d\n", msg.data[0]);
            led2 = !led2;
        } 
        wait(0.2);
    }
}

#endif

#if MBED_MEM_BLOCK_DEVICE_H_EXAMPLE


/** Lazily allocated heap-backed block device
 *
 * Useful for simulating a block device and tests
 *
 * @code
 * #include "mbed.h"
 * #include "HeapBlockDevice.h"
 *
 * #define BLOCK_SIZE 512
 *
 * HeapBlockDevice bd(2048, BLOCK_SIZE); // 2048 bytes with a block size of 512 bytes
 * uint8_t block[BLOCK_SIZE] = "Hello World!\n";
 *
 * int main() {
 *     bd.init();
 *     bd.erase(0, BLOCK_SIZE);
 *     bd.program(block, 0, BLOCK_SIZE);
 *     bd.read(block, 0, BLOCK_SIZE);
 *     printf("%s", block);
 *     bd.deinit();
 * }
 * @endcode
 */
class HeapBlockDevice : public BlockDevice
{
public:

    /** Lifetime of the memory block device
     *
     * @param size      Size of the Block Device in bytes
     * @param block     Block size in bytes. Minimum read, program, and erase sizes are
     *                  configured to this value
     */
    HeapBlockDevice(bd_size_t size, bd_size_t block=512);
    /** Lifetime of the memory block device
     *
     * @param size      Size of the Block Device in bytes
     * @param read      Minimum read size required in bytes
     * @param program   Minimum program size required in bytes
     * @param erase     Minimum erase size required in bytes
     */
    HeapBlockDevice(bd_size_t size, bd_size_t read, bd_size_t program, bd_size_t erase);
    virtual ~HeapBlockDevice();

    /** Initialize a block device
     *
     *  @return         0 on success or a negative error code on failure
     */
    virtual int init();

    /** Deinitialize a block device
     *
     *  @return         0 on success or a negative error code on failure
     */
    virtual int deinit();

    /** Read blocks from a block device
     *
     *  @param buffer   Buffer to read blocks into
     *  @param addr     Address of block to begin reading from
     *  @param size     Size to read in bytes, must be a multiple of read block size
     *  @return         0 on success, negative error code on failure
     */
    virtual int read(void *buffer, bd_addr_t addr, bd_size_t size);

    /** Program blocks to a block device
     *
     *  The blocks must have been erased prior to being programmed
     *
     *  @param buffer   Buffer of data to write to blocks
     *  @param addr     Address of block to begin writing to
     *  @param size     Size to write in bytes, must be a multiple of program block size
     *  @return         0 on success, negative error code on failure
     */
    virtual int program(const void *buffer, bd_addr_t addr, bd_size_t size);

    /** Erase blocks on a block device
     *
     *  The state of an erased block is undefined until it has been programmed
     *
     *  @param addr     Address of block to begin erasing
     *  @param size     Size to erase in bytes, must be a multiple of erase block size
     *  @return         0 on success, negative error code on failure
     */
    virtual int erase(bd_addr_t addr, bd_size_t size);

    /** Get the size of a readable block
     *
     *  @return         Size of a readable block in bytes
     */
    virtual bd_size_t get_read_size() const;

    /** Get the size of a programable block
     *
     *  @return         Size of a programable block in bytes
     */
    virtual bd_size_t get_program_size() const;

    /** Get the size of a eraseable block
     *
     *  @return         Size of a eraseable block in bytes
     */
    virtual bd_size_t get_erase_size() const;

    /** Get the total size of the underlying device
     *
     *  @return         Size of the underlying device in bytes
     */
    virtual bd_size_t size() const;

private:
    bd_size_t _read_size;
    bd_size_t _program_size;
    bd_size_t _erase_size;
    bd_size_t _count;
    uint8_t **_blocks;
};


#endif
