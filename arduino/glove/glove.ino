#include <SPI.h>  

const int ChipSelPin1 = 4; // On the ArduIMU+ V3, output pin D4 on the ATmega328P connects to
unsigned int packetSize = 42; // number of unique bytes of data written by the DMP each time (FIFO can hold multiples of 42-bytes)
unsigned int fifoCount;       // count of all bytes currently in FIFO
byte fifoBuffer[64];          // FIFO storage buffer (in fact only 42 used...)

int channel[5];
int flxs1, flxs2, flxs3, flxs4, flxs5, flxst;

void setup(){
  Serial.begin(115200);

  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A6, HIGH);

  // Flush serial buffer to clean up remnants from previous run
  while(Serial.available() > 0) Serial.read(); 
  Serial.println();
    
  //--- SPI settings ---//
  Serial.println("Initializing SPI Protocol...");
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16); 
  SPI.setBitOrder(MSBFIRST);  // data delivered MSB first as in MPU-6000 Product Specification
  SPI.setDataMode(SPI_MODE0); // latched on rising edge, transitioned on falling edge, active low
  delay(100);
  
  pinMode(ChipSelPin1, OUTPUT);
    
  // write & verify dmpMemory, dmpConfig and dmpUpdates into the DMP, and make all kinds of other settings
  Serial.println("Initializing Digital Motion Processor (DMP)...");
  byte devStatus; // return status after each device operation (0 = success, !0 = error)

}

void loop() {
  byte mpuIntStatus = SPIread(0x3A, ChipSelPin1); // by reading INT_STATUS register, all interrupts are cleared
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1008) {
    SPIwriteBit(0x6A, 2, true, ChipSelPin1); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
  } else if (mpuIntStatus & 0x02) {
    
    SPIreadBytes(0x74, packetSize, fifoBuffer, ChipSelPin1);
    fifoCount = fifoCount - packetSize;

    int AcceX = ((fifoBuffer[28] << 8) + fifoBuffer[29]);
    int AcceY = ((fifoBuffer[32] << 8) + fifoBuffer[33]);
    int AcceZ = ((fifoBuffer[36] << 8) + fifoBuffer[37]);
    float ADegX = (atan2(AcceY, AcceZ) + PI) * RAD_TO_DEG;
    float ADegY = (atan2(AcceX, AcceZ) + PI) * RAD_TO_DEG;


    flexRead();
    int flxs1 = (int)channel[0];
    int flxs2 = (int)channel[1];
    int flxs3 = (int)channel[2];
    int flxs4 = (int)channel[3];
    int flxs5 = (int)channel[4];
    unsigned int flxst = (flxs1 + flxs5 + flxs3 + flxs4) / 4;
    flxst = map(flxst, 490, 625, 1000, 1600);

    Serial.print(flxst); // 0
    Serial.print(",");
    
    Serial.print(ADegX - 180); // 1
    Serial.print(",");
    Serial.print(ADegY - 180); // 2
    Serial.print(",");

    Serial.print(AcceX); // 3
    Serial.print(",");
    Serial.print(AcceY); // 4
    Serial.print(",");
    Serial.println(AcceZ); // 5
  }
}

byte SPIread(byte reg, int ChipSelPin) {
  digitalWrite(ChipSelPin, LOW);     // select MPU-6000 for SPI transfer (low active)
  SPI.transfer(reg | 0x80);          // reg | 0x80 causes a "1" added as MSB to reg to denote reading from reg i.s.o. writing to it
  byte read_value = SPI.transfer(0x00); // write 8-bits zero to MPU-6000, read the 8-bits coming back from reg at the same time
  digitalWrite(ChipSelPin, HIGH);    // deselect MPU-6000 for SPI transfer
  return read_value;
}

void SPIwrite(byte reg, byte data, int ChipSelPin) {
  digitalWrite(ChipSelPin, LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(ChipSelPin, HIGH);
}

void SPIwriteBit(byte reg, byte bitNum, byte databit, int ChipSelPin) {
  byte byte_value = SPIread(reg, ChipSelPin);
  if (databit == 0) {
    byte_value = byte_value & ~(1 << bitNum); // AND result from register byte value and byte with only one "0" at place of bit to write (rest "1"'s)
  } else {
    byte_value = byte_value |  (1 << bitNum); // OR  result from register byte value and byte with only one "1" at place of bit to write (rest "0"'s)
  }
  SPIwrite(reg, byte_value, ChipSelPin);
}

void SPIreadBytes(byte reg, unsigned int length, byte *data, int ChipSelPin) {
  digitalWrite(ChipSelPin, LOW);
  delay(10); // wait 10 ms for MPU-6000 to react on chipselect (if this is 4 ms or less, SPI.transfer fails)
  SPI.transfer(reg | 0x80); // reg | 0x80 causes a "1" added as MSB to reg to denote reading from reg i.s.o. writing to it

  unsigned int count = 0;
  byte data_bytes_printed = 0;

  for (count = 0; count < length; count ++) {
    data[count] = SPI.transfer(0x00);
  }
  digitalWrite(ChipSelPin, HIGH);
}

void flexRead() {
  channel[0] = analogRead(A0);
  channel[1] = analogRead(A1);
  channel[2] = analogRead(A2);
  channel[3] = analogRead(A3);
  channel[4] = analogRead(A6);
}
