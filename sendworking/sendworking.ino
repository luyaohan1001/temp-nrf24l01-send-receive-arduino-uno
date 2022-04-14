#define TX_ADR_WIDTH  5        
#define RX_ADR_WIDTH  5       
#define TX_PLOAD_WIDTH  32  
#define RX_PLOAD_WIDTH  32  

//#define TX_ADDRESS   0x0909090909 
//#define RX_ADDRESS   0x0909090909 

uint8_t TX_ADDRESS[5] = {0xE6, 0xE6, 0xE6, 0xE6, 0xE6};
uint8_t RX_ADDRESS[5] = {0xE6, 0xE6, 0xE6, 0xE6, 0xE6};
//#define TX_ADDRESS   0x0  //本地地址
//#define RX_ADDRESS   0x0  //接收地址
#define R_REGISTER_MASK   0x00                   
#define W_REGISTER_MASK   0x20
#define R_RX_PAYLOAD  0x61                      
#define W_TX_PAYLOAD  0xA0                       
#define FLUSH_TX     0xE1                       
#define FLUSH_RX     0xE2                      
#define REUSE_TX_PL  0xE3                       
#define NOP          0xFF                      

// *************************************SPI(nRF24L01)寄存器地址****************************************************
// Mnemonic   Address  Description
#define CONFIG        0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA         0x01  // 自动应答功能设置
#define EN_RXADDR     0x02  // 可用信道设置
#define SETUP_AW      0x03  // 收发地址宽度设置
#define SETUP_RETR    0x04  // 自动重发功能设置
#define RF_CH         0x05  // 工作频率设置
#define RF_SETUP      0x06  // 发射速率、功耗功能设置
#define STATUS        0x07  // 状态寄存器
#define OBSERVE_TX    0x08  // 发送监测功能
#define CD            0x09  // 地址检测           
#define RX_ADDR_P0    0x0A  // 频道0接收数据地址
#define RX_ADDR_P1    0x0B  // 频道1接收数据地址
#define RX_ADDR_P2    0x0C  // 频道2接收数据地址
#define RX_ADDR_P3    0x0D  // 频道3接收数据地址
#define RX_ADDR_P4    0x0E  // 频道4接收数据地址
#define RX_ADDR_P5    0x0F  // 频道5接收数据地址
#define TX_ADDR       0x10  // 发送地址寄存器
#define RX_PW_P0      0x11  // 接收频道0接收数据长度
#define RX_PW_P1      0x12  // 接收频道0接收数据长度
#define RX_PW_P2      0x13  // 接收频道0接收数据长度
#define RX_PW_P3      0x14  // 接收频道0接收数据长度
#define RX_PW_P4      0x15  // 接收频道0接收数据长度
#define RX_PW_P5      0x16  // 接收频道0接收数据长度
#define FIFO_STATUS   0x17  // FIFO栈入栈出状态寄存器设置
#define TX_OK         0x20  //TX发送完成中断
#define MAX_TX        0x10  //达到最大发送次数中断


#define MOSI   11
#define CSN    8
#define MISO   12
#define SCK    13
#define CE     9

// Pin Wiggling Macros:
void SPI_CS_1() {
    digitalWrite(CSN, LOW); 
}
void SPI_CS_0(){
    digitalWrite(CSN, HIGH); 
}
void SPI_SCK_1(){
    digitalWrite(SCK, HIGH);
}
void SPI_SCK_0(){
    digitalWrite(SCK, LOW);
} 
void SPI_MOSI_1(){
    digitalWrite(MOSI, HIGH);
}
void SPI_MOSI_0(){
    digitalWrite(MOSI, LOW); 
}
void SPI_CE_1(){
    digitalWrite(CE, HIGH); 
}
void SPI_CE_0(){
    digitalWrite(CE, LOW); 
}
    
uint8_t SPI_READ_MISO(){
  return digitalRead(MISO);
}



void gpio_init(){
  pinMode(MISO, INPUT); 
  
  pinMode(SCK, OUTPUT); 
  pinMode(MOSI, OUTPUT); 
  pinMode(CE, OUTPUT); 
  pinMode(CSN, OUTPUT); 
  SPI_CS_0();
  SPI_SCK_0();
}


void gpio_clockout_8_bits(uint8_t txData) {
  spi_delay();
  for (int i = 0; i < 8; ++i) {
      SPI_SCK_0();
      spi_delay();
      if(txData & 0x80) // MSBit first
          SPI_MOSI_1();
      else
          SPI_MOSI_0();
      SPI_SCK_1(); // clock data
      txData = txData << 1; // load next MSB
      spi_delay();
  }
  SPI_SCK_0();
}

uint8_t gpio_clockin_8_bits(){
  uint8_t rxData = 0;
  spi_delay();
  for (int i=0; i < 8; ++i) {
      SPI_SCK_0();
      spi_delay();
      // SPI_MOSI_0() # dummy byte - dummy bit, clock edge is sufficient for MISO data to spit out.
      SPI_SCK_1();
      spi_delay();
      rxData = rxData << 1; // Why shift first then OR'? range (0, 8) will need to shift only 7 times.
      rxData |= SPI_READ_MISO();
      spi_delay();
  }
  SPI_SCK_0();
  return rxData;
}
  
void spi_delay() {
  //delay(1);
}

void spi_write_register(uint8_t reg, uint8_t num_bytes, uint8_t* val){
  // Select chip
  if (num_bytes == 4) {
    //Serial.print((uint8_t) val[0], HEX);
    //Serial.print((uint8_t)val[1], HEX);
    //Serial.print((uint8_t)val[2], HEX);
    //Serial.print((uint8_t)val[3], HEX);
  }
  SPI_CS_1();

  // Write chip register 
  gpio_clockout_8_bits(reg);
  // Write value
  for (int i = 0; i < num_bytes; ++i){
    uint8_t writing_byte = val[i];
    gpio_clockout_8_bits(writing_byte);
    // val = val >> 8;
    
  }

  // Deselect chip
  SPI_CS_0();
}

void spi_read_register(uint8_t reg, uint8_t num_bytes, uint8_t* pbuf){
  // Select chip
  SPI_CS_1();
  
  // Write register address to read.
  gpio_clockout_8_bits(reg);
  // Read value
  for (int i = 0; i < num_bytes; ++i) {
    pbuf[i] = gpio_clockin_8_bits();
  }
  
  // Deselect chip
  SPI_CS_0();
}

// uint8_t tx_addr[5];
// spi_read_register(R_REGISTER_MASK + TX_ADDR, 5, tx_addr);
// Serial.print("tx_addr: "); for (int i = 0; i < 5; i++) Serial.print(tx_addr[i], HEX); Serial.println();
uint8_t nrf24_get_STATUS() {
  uint8_t stat;
  spi_read_register(R_REGISTER_MASK + STATUS, 1, &stat);
  // Serial.println("--------");
  // Serial.print("STATUS: "); Serial.println(stat,HEX);
  return stat;
}

uint8_t nrf24_get_FIFO_STATUS() {
  uint8_t fifo_status;
  spi_read_register(R_REGISTER_MASK + FIFO_STATUS, 1, &fifo_status);
  // Serial.print("- FIFO STATUS: "); Serial.println(fifo_status,HEX);
}


uint8_t nrf24_get_CONFIG() {
  uint8_t config_reg;
  spi_read_register(R_REGISTER_MASK + CONFIG, 1, &config_reg);
  // Serial.print("- CONFIG: "); Serial.println(config_reg,HEX);
}


/**
 * Brief: Before we start debugging whether Transmitter can send or Receiver can receive. We need to make sure that SPI can write to nRF24 registers successfully.
 * Test: Write to a bunch of registers and read. Compare if read value is same as write value.
 */
void nrf24_spi_write_test() {
    Serial.println("---- nRF24 SPI registers write test. ----");
    
    uint8_t write_bytes[5] = {0x10,0x10,0x10,0x10,0x10};  // 定义一个静态发送地址
    uint8_t original_bytes[5] = {0};
    uint8_t read_bytes[5] = {0};
    spi_read_register(R_REGISTER_MASK + TX_ADDR, 5, original_bytes);  // 为了应答接收设备，接收通道0地址和发送地址相同
    spi_write_register(W_REGISTER_MASK + TX_ADDR, 5, write_bytes);     // 写入发送地址
    spi_read_register(R_REGISTER_MASK + TX_ADDR, 5, read_bytes);  // 为了应答接收设备，接收通道0地址和发送地址相同
    spi_write_register(W_REGISTER_MASK + TX_ADDR, 5, original_bytes);     // 写入发送地址
    
    if (read_bytes[0] == write_bytes[0]
      && read_bytes[1] == write_bytes[1]
      && read_bytes[2] == write_bytes[2]
      && read_bytes[3] == write_bytes[3]
      && read_bytes[4] == write_bytes[4]
    ) {
      Serial.println("SPI write continuous bytes test passed.");
    } else {
      Serial.println("Problem writing continuous bytes through SPI. ");
    }

    
    uint8_t write_byte = 0x01;
    uint8_t original_byte = 0x00;
    uint8_t read_byte = 0x00;
    spi_read_register(R_REGISTER_MASK + EN_AA, 1, &original_byte);
    spi_write_register(W_REGISTER_MASK + EN_AA, 1, write_byte);      
    spi_read_register(R_REGISTER_MASK + EN_AA, 1, &read_byte);
    spi_write_register(W_REGISTER_MASK + EN_AA, 1, original_byte);      

    if (read_byte == write_byte) {
      Serial.println("SPI write single byte test passed.");
    } else {
      Serial.println("Problem writing single bytes through SPI. ");
    }
    
}

/**  
 *  Brief: We need to make sure the Transmitter work before we have a Receiver working.
 *  
 *  How to know TX payload is loaded?
 *  
 *  Steps: 1. Disable Auto Acknowledgement, Auto Retransmit. TX_DS will be be set if these two are not turn-off.
 *         2. TX_DS (in STATUS register) is expected to be set when data in TX FIFO is set.
 *         STATUS = 0x2E after firing means we have a working TX module.
 *        
 *        After writing to W_TX_PAYLOAD, TX_EMPTY (in FIFO_STATUS register) becomes 0.
 *        
 *        What happens if sending is not successful?
 *        TX_FULL (in FIFO_STATUS register) becomes 1.
 *        TX_FULL (in STATUS register) becomes 1.
 *        TX_DS (in STATUS register) remains 0.  
 *  States: 
 *        The states can be referred in 6.1.1 State diagram.
 *  
 *  STATUS register
 *            7             6           5         4         3:1         0
 *        <reserved>      RX_DR       TX_DS     MAX_RT     RX_P_NO     TX_FULL
 *        -----------------------------------------------------------------------
 *         Always 0       Receive     Transfer  Maximum    000-101 :    1: 
 *                        Data        Data      TX         Data Pipe #  TX_FULL
 *                        Ready       Sent      Transmits  110 :
 *                                                         Not Used.
 *                                                         111:
 *                                                         RX FIFO Empty.
 */
bool nrf24_tx_self_test() {
  Serial.println("---- nRF24 TX self test. ----");
  Serial.println("---- A test to verify SPI register settings. Send without a receiver. ----");
  uint8_t nrf24_status = 0x00;

  // Note that if we reset the Arduino without re-poweron the chip, initial value of registers such as STATUS or CONFIG may be different from one listed in datasheet.
  // [Current State: Power-on reset 100 ms] 
  SPI_CE_0();
  // [Current State: (RF transmission is) Power Down (But SPI is alive.)]
  spi_write_register(W_REGISTER_MASK + EN_AA, 1, 0x00);        // disable auto acknowledgement  
  spi_write_register(W_REGISTER_MASK + EN_RXADDR, 1, 0x00);    // disable RX data pipes
  spi_write_register(W_REGISTER_MASK + SETUP_RETR, 1, 0x00);   // disable automatic re-transmit, ARC = 0000
  spi_write_register(W_REGISTER_MASK + CONFIG, 1, 0x0E);       // PWR_UP = 1 PRIMRX=0 (TX mode)

  // PWR_UP=1 -> [Current State: Standby-I]
  uint8_t test_payload[4] = {0xDE, 0xAD, 0xBE, 0xEF}; // clock in a payload, now TX FIFO not empty 
  spi_write_register(W_TX_PAYLOAD, 4, test_payload);
  SPI_CE_1(); // Chip Enable. Fire the packet out on the antenna!
  
  // TX FIFO not empty / CE = 1 -> [Current State: TX MODE]
  nrf24_status = nrf24_get_STATUS();
  delay(1);

  // Now check if TX self-test pass.
  nrf24_status = nrf24_get_STATUS();  
  if (nrf24_status & 0x2E) { // TX_DS bit is set.
    Serial.println("");
    Serial.println(" > nrf24 transmission self test has passed. STATUS has value of 0x2E, TX_DS (transfer data sent) was set, RX_P_NO = 111, means RX FIFO Empty.");
    return true;
  } else {
    Serial.println("");
    Serial.print("nrf24 transmission self test has failed. STATUS is expected 0x2E. The actual value is: "); Serial.println(nrf24_status);
    return false;
  }
  SPI_CE_0();
  // CE=0 -> Return to [State: Standby-I]. 
  uint8_t val = 0x08;
  spi_write_register(W_REGISTER_MASK + CONFIG, 1, &val);
  val = 0x0e;
  spi_write_register(W_REGISTER_MASK + STATUS, 1, &val);
  // PWR_UP=0 -> Return to Power-Down Mode.
}



void nrf24_keep_sending() {

  uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF}; // clock in a payload, TX FIFO not empty 
  spi_write_register(W_TX_PAYLOAD, 4, (uint8_t*) payload);
  SPI_CE_1(); // fire out the transmit packet
  delay(1);
  // TX Setting
  uint8_t stat = nrf24_get_STATUS();
  Serial.print(stat, HEX);
  // if (stat & 0x20) { // TX_DS bit is set.
  if (stat == 0x2e) { // TX_DS bit is set.
    // Serial.println("nrf24 send successful.");
  } else {
    // Serial.println("nrf24 send failure.");
  }
  // write 1 to clear TX_DS
  spi_write_register(W_REGISTER_MASK + STATUS, 1, 0x20); 
  SPI_CE_0(); // stop transmission. return to Standby-I state.
  // Return to [State: Standby-I]
}


void RF_SEND() {
    
    uint8_t val;
    uint8_t check;
    uint8_t cfg = 0;
    spi_read_register(R_REGISTER_MASK + CONFIG, 1, &cfg);
    Serial.print("config: "); Serial.println(cfg, HEX);
  
    SPI_CE_0();

    unsigned char TX_ADDRESS[5] = {0x10,0x10,0x10,0x10,0x10};  // 定义一个静态发送地址
    spi_write_register(W_REGISTER_MASK + TX_ADDR, 5, TX_ADDRESS);     // 写入发送地址
    spi_write_register(W_REGISTER_MASK + RX_ADDR_P0, 5, TX_ADDRESS);  // 为了应答接收设备，接收通道0地址和发送地址相同
    

    uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF}; // clock in a payload, TX FIFO not empty 

    spi_write_register(W_TX_PAYLOAD, 4, payload);                  // 写数据包到TX FIFO
    
    val = 0x00;
    spi_write_register(W_REGISTER_MASK + EN_AA, 1, &val);       // 使能接收通道0自动应答
    spi_read_register(R_REGISTER_MASK + EN_AA, 1, &check);
    if (check != val) { Serial.print("We have problem with EN_AA. "); Serial.print("shoud be: "); Serial.print(val,HEX); Serial.print(" but it is: ");Serial.println(check, HEX); }

    val = 0x00;
    spi_write_register(W_REGISTER_MASK + EN_RXADDR, 1, &val);   // 使能接收通道0
    spi_read_register(R_REGISTER_MASK + EN_RXADDR, 1, &check);
    if (check != val) Serial.println("We have problem with EN_RXADDR");

    val = 0x00;
    spi_write_register(W_REGISTER_MASK + SETUP_RETR, 1, &val);  // 自动重发延时等待250us+86us，自动重发10次
    spi_read_register(R_REGISTER_MASK + SETUP_RETR, 1, &check);
    if (check != val) Serial.println("We have problem with SETUP_RETR");

    val = 40;
    spi_write_register(W_REGISTER_MASK + RF_CH, 1, &val);         // 选择射频通道0x40
    spi_read_register(R_REGISTER_MASK + RF_CH, 1, &check);
    if (check != val) { Serial.println("We have problem with RF_CH"); Serial.print("shoud be: "); Serial.print(val,HEX); Serial.print(" but it is: ");Serial.println(check, HEX); }

    val = 0x07;
    spi_write_register(W_REGISTER_MASK + RF_SETUP, 1, &val);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
    spi_read_register(R_REGISTER_MASK + RF_SETUP, 1, &check);
    if (check != val) { Serial.println("We have problem with RF_SETUP"); Serial.print("shoud be: "); Serial.print(val,HEX); Serial.print(" but it is: ");Serial.println(check, HEX); }
    
    val = 0x0e;
    spi_write_register(W_REGISTER_MASK + CONFIG, 1, &val);      // CRC使能，16位CRC校验，上电
    spi_read_register(R_REGISTER_MASK + CONFIG, 1, &check);
    if (check != val) Serial.println("We have problem with CONFIG");
    delay(150);
    
    uint8_t stat = nrf24_get_STATUS();
    Serial.print(stat, HEX);
    // if (stat & 0x20) { // TX_DS bit is set.
    if (stat == 0x2e) { // TX_DS bit is set.
      Serial.println("nrf24 send successful.");
    } else {
      // Serial.println("nrf24 send failure.");
    }
    
    SPI_CE_1();


}

void setup() {
  Serial.begin(9600); 
  gpio_init();

  nrf24_spi_write_test();
  // nrf24_tx_self_test();
  


  
}

void loop() {
  while(1) {
    // nrf24_keep_sending();
    RF_SEND();
    delay(1000);
  
  }
}
