/*
 * 
 * Arduino Sketch to communicate with the Syma S107 IR Helicopter.
 * Author: Vivin Suresh Paliath
 *
 * This code has been adapted from the code available at the following URL:
 *
 *   https://sites.google.com/site/spirixcode/code/kodek.txt?attredirects=0
 *
 * The original author is Jim Hamilton AKA "Aqualung", from the RCGroups.com message board.
 *
 * Polarities have been reversed from the original version and the code has been modified to be more readable. In
 * addition, comments have been added.
 *
 * IR protocol is as follows:
 *
 *     Yaw     Pitch   Throttle   Trim
 *    Byte 1   Byte 2   Byte 3   Byte 4
 * H 0YYYYYYY 0PPPPPPP CTTTTTTT 0AAAAAAA F
 *
 * H: Header
 * Y: Yaw
 * P: Pitch
 * C: Channel
 * T: Throttle
 * A: Adjustment (Trim)
 * F: Footer
 *
 *
 * The transmitter sends a continuous stream of bits at a frequency of 38 KHz. When transmitting a byte, the 
 * transmitter starts with the most-significant bit.
 *
 * Header:  77 cycles from high to low (13us high, 13us low) giving a total of 2002us followed by 1998us low.
 * Command: 12 cycles from high to low (13us high, 13us low) giving a total of 338us followed by 688us low for a 1 or
 *          288us low for a 0. This is repeated for each bit (meaning, each bit is preceded by the 12 cycles).
 * Footer:  12 cycles from high to low (13us high, 13us low) giving a total of 338us. This marks the end of one
 *          "packet".
 *
 *
 * The main loop consumes data from a python script that will send data only when the arduino needs it. To tell the
 * script that it needs data, the arduino sends a byte (READY_TO_ACCEPT_ACK) back through the serial channel. When
 * the script sees this byte, it will send a packet of data. This way, we don't lose any data.
 */


/* Arduino pin #defines */
#define LED 8
#define STATUS 13

/* 
 * Human-readable names for the subscripts of the packet byte-array where each subscript represents a yaw byte,
 * pitch byte, throttle byte, or a trim byte
 */

#define YAW      0
#define PITCH    1
#define THROTTLE 2
#define TRIM     3

/*
 * A few useful #defines
 */

#define BYTES_IN_PACKET    4
#define BITS_IN_BYTE       8

#define PACKETS_PER_SECOND       10
#define MICROSECONDS_IN_A_SECOND 1000

/*
 * Human-readable values for yaw, pitch, and trim
 */
 
#define ZERO_YAW      63
#define ZERO_PITCH    63
#define ZERO_THROTTLE 0
#define ZERO_TRIM     63

/*
 * Human-readable names for various components of the signal, such as the number of cycles, and the duration of
 * signals.
 */

#define CYCLES_FOR_HEADER 77
#define CYCLES_FOR_BIT    12
#define CYCLES_FOR_FOOTER 12
#define CYCLE_TIME      26

#define HEADER_DELAY 1998
#define ONE          688
#define ZERO         288

/*
 * Masks for bits. These masks help us easily examine the value of a bit at a specific bit-position so that we can
 * see if it is 0 or 1. 
 *
 * For example, assume that we have the following bit pattern:
 *
 *   Bits
 * 76543210
 * --------
 * 10100101
 *
 *
 * Let's say that we want to examine the bit at position 5. From visually inspecting the value, we can see that it
 * is a 1. To get that specific value, we will need to AND the value with 32 (2 ^ 5), which in binary is 00100000:
 *
 * 1010 0101
 *     &
 * 0010 0000
 * ---------
 * 0010 0000
 *   ^
 *   |
 *   +------------- The bit in position 5 is a 1.
 *
 *
 * Similarly, let's say that we wanted to examine the bit at position 1. We will then need to AND the value with 2
 * (2 ^ 1), which in binary is 00000010:
 *
 * 1010 0101
 *     &
 * 0000 0010
 * ---------
 * 0000 0000
 *        ^
 *        |
 *        +-------- The bit in position 1 is a 0.
 *
 */

#define BIT_0 1   // 2 ^ 0 = 1   = 0b00000001
#define BIT_1 2   // 2 ^ 1 = 2   = 0b00000010
#define BIT_2 4   // 2 ^ 2 = 4   = 0b00000100
#define BIT_3 8   // 2 ^ 3 = 8   = 0b00001000
#define BIT_4 16  // 2 ^ 4 = 16  = 0b00010000
#define BIT_5 32  // 2 ^ 5 = 32  = 0b00100000
#define BIT_6 64  // 2 ^ 6 = 64  = 0b01000000
#define BIT_7 128 // 2 ^ 7 = 128 = 0b10000000

/* 
 * The python program sends data close to twice as fast as the arduino can consume it. So let's tell the python program
 * when we're ready to accept data. To do this, we'll send a byte across the serial channel that will let the script know
 * that we're ready to accept data.
 */
 
#define READY_TO_ACCEPT_ACK 129

/* This array will hold data that we read in from the serial port */
byte command[4] = {ZERO_YAW, ZERO_PITCH, ZERO_THROTTLE, ZERO_TRIM};

void setup() {
   Serial.begin(9600);
   
   pinMode(STATUS, OUTPUT);
   pinMode(LED, OUTPUT);
  
   digitalWrite(STATUS, LOW);
   digitalWrite(LED, LOW);
}

int sendPacket(byte yaw, byte pitch, byte throttle, byte trim) {
  
   digitalWrite(STATUS, HIGH);

   static int current_byte_index, current_bit_index, ones, zeroes;
   static const byte bit_masks[] = {BIT_0, BIT_1, BIT_2, BIT_3, BIT_4, BIT_5, BIT_6, BIT_7};
   static byte packet_bytes[4];
   
   ones = 0;
   zeroes = 0;
   
   packet_bytes[YAW] = yaw;
   packet_bytes[PITCH] = pitch;
   packet_bytes[THROTTLE] = throttle;
   packet_bytes[TRIM] = trim;

   current_byte_index = 0;
   current_bit_index = BITS_IN_BYTE - 1;

   //Send the header
   header();

   //We will iterate over each byte in our packet
   while(current_byte_index < BYTES_IN_PACKET) {
     
      //Send the pulses that signify the start of a "bit"
      pulse(CYCLES_FOR_BIT);

      //Delay for the appropriate amount of time, depending on whether the bit in question is
      //a 0 or a 1
      if(packet_bytes[current_byte_index] & bit_masks[current_bit_index--]) {
         one();
         ones++;
      } else {
         zero();
         zeroes++;
      }

      //If current_bit_index is lesser than zero, it means that we have sent all the bits for
      //this byte. So, we need to reset current_bit_index and increment the value of
      //current_byte_index
      if(current_bit_index < 0) {
         current_bit_index = BITS_IN_BYTE - 1;
         current_byte_index++;
      }
   }

   //Send the footer
   footer();
   
   digitalWrite(STATUS, LOW);

   //We want to return the variable delay it takes to send 10 packets a second. 1 second is 1000ms.
   //which means that each packet takes 100ms to send. Our packet will take nowhere near that time
   //to send; it will fall short. So we just need to calculate the total time it takes to send the
   //packet in microseconds and then subtract that value from 100ms, to get our variable delay
 
   int header_time = (CYCLES_FOR_HEADER * CYCLE_TIME) + HEADER_DELAY;
   int footer_time = (CYCLES_FOR_FOOTER * CYCLE_TIME);
   int total_bit_start_time = ((CYCLES_FOR_BIT * CYCLE_TIME) * BITS_IN_BYTE) * BYTES_IN_PACKET;
   int ones_time = ONE * ones;
   int zeroes_time = ZERO * zeroes;
   int total_packet_time = header_time + footer_time + total_bit_start_time + ones_time + zeroes_time;

   return (MICROSECONDS_IN_A_SECOND / PACKETS_PER_SECOND) - ((float) total_packet_time / MICROSECONDS_IN_A_SECOND);
}

void header() {
   pulse(CYCLES_FOR_HEADER);
   delayMicroseconds(HEADER_DELAY);
}

void footer() {
   pulse(CYCLES_FOR_FOOTER);
}

void pulse(int cycles) {
   while(cycles > 0) {
      digitalWrite(LED, HIGH); //This takes 3us
      delayMicroseconds(10);   //Wait 10us so that we have a high pulse that is 13us long
      
      digitalWrite(LED, LOW);  //This also takes 3us
      delayMicroseconds(10);   //Wait 10us so that we have a low pulse that is 13us long

      cycles--;
   }
}

void one() {
   delayMicroseconds(ONE);
}

void zero() {
   delayMicroseconds(ZERO);
}

void loop() {
  
   while(1) {
     
      /* If data is available in the serial channel, let's read it */
      if(Serial.available() >= BYTES_IN_PACKET) {      
         for(int i = 0; i < 4; i++) {
            command[i] = Serial.read();
         }
      }   

      /* Send data to the helicopter */
      delay(sendPacket(command[YAW], command[PITCH], command[THROTTLE], command[TRIM]));
      
      /* Let the python script know that we are ready to accept new data */
      Serial.write(READY_TO_ACCEPT_ACK);
   }
}
