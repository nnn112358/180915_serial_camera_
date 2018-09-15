//
// Grove Serial Camera Kit demo for M5Stack
//
// !! CAUTION !!
// This code uses M5Stack Grove I2C as Grove UART (serial port).
// REMOVE all I2C devices.
//
// original code:
// https://github.com/Seeed-Studio/Grove_Serial_Camera_Kit/blob/master/SerialCameral_DemoCode_CJ_OV528_SoftSer/SerialCameral_DemoCode_CJ_OV528_SoftSer.ino
//  File SerialCamera_DemoCode_CJ-OV528.ino
//  8/8/2013 Jack Shao
//  Demo code for using seeeduino or Arduino board to cature jpg format
//  picture from seeed serial camera and save it into sd card. Push the
//  button to take the a picture .

//  For more details about the product please check http://www.seeedstudio.com/depot/

#include <M5Stack.h>
HardwareSerial Serial2_camera(2);

#define PIC_PKT_LEN    128        //data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0

#define PIC_FMT        PIC_FMT_CIF

File myFile;

const byte cameraAddr = (CAM_ADDR << 5);  // addr
//const int buttonPin = A5;                 // the number of the pushbutton pin
unsigned long picTotalLen = 0;            // picture length
int picNameNum = 0;

#define BPS115200_A 0x0F
#define BPS115200_B 0x01

void setBaud(char BPS_A, char BPS_B) {
  char cmd[] = { 0xaa, 0x07, BPS_A, BPS_B, 0, 0};
  //char cmd[] = {0xaa, 0x0d | cameraAddr, BPS_A, BPS_B, 0x00, 0x00} ;
  Serial.println("setBaud");
  unsigned char resp[6];
  Serial2_camera.setTimeout(100);
  while (1) {
    clearRxBuf();
    sendCmd(cmd, 6);
    uint8_t x = Serial2_camera.readBytes((char *)resp, 6);
    if (x != 6) continue;
    if (resp[0] == 0xaa && resp[1] == 0x0e && resp[2] == 0x07 && resp[4] == 0 && resp[5] == 0) {
      break;
    }
    Serial.println(cmd);
  }
}
/*********************************************************************/
void setup()
{

  M5.begin();
  M5.Lcd.setBrightness(200);
  Serial.begin(115200);


  Serial2_camera.begin(9600, SERIAL_8N1, 22, 21); //cant be faster than 9600, maybe difference with diff board.
  setBaud(BPS115200_A, BPS115200_B);
  Serial2_camera.end();
 Serial2_camera.begin(115200, SERIAL_8N1, 22, 21); //cant be faster than 9600, maybe difference with diff board.
 
  initialize();


}
int n = 0;
/*********************************************************************/
void loop()
{
  if (M5.BtnA.wasPressed()) {
    if (n == 0) preCapture();
    Capture();
    GetData();
    Serial.print("\r\nDone ,number : ");
    Serial.println(n);
    n++ ;
  }
  M5.update();
}
/*********************************************************************/
void clearRxBuf()
{
  while (Serial2_camera.available())
  {
    Serial2_camera.read();
  }
}
/*********************************************************************/
void sendCmd(char cmd[], int cmd_len)
{
  for (char i = 0; i < cmd_len; i++) Serial2_camera.write(cmd[i]);
}
/*********************************************************************/
int readBytes(char *dest, int len, unsigned int timeout)
{
  int read_len = 0;
  unsigned long t = millis();
  while (read_len < len)
  {
    while (Serial2_camera.available() < 1)
    {
      if ((millis() - t) > timeout)
      {
        return read_len;
      }
    }

    //

    *(dest + read_len) = Serial2_camera.read();
    //Serial.write(*(dest+read_len));
    read_len++;
  }
  return read_len;
}
/*********************************************************************/
void initialize()
{
  char cmd[] = {0xaa, 0x0d | cameraAddr, 0x00, 0x00, 0x00, 0x00} ;
  //    char cmd[] = {0xaa, 0x0d | cameraAddr, BPS_A, BPS_B, 0x00, 0x00} ;

  unsigned char resp[6];

  Serial.print("initializing camera...");

  while (1)
  {
    sendCmd(cmd, 6);
    if (readBytes((char *)resp, 6, 1000) != 6)
    {
      Serial.print(".");
      continue;
    }
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
    {
      if (readBytes((char *)resp, 6, 500) != 6) continue;
      if (resp[0] == 0xaa && resp[1] == (0x0d | cameraAddr) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;
    }
  }
  cmd[1] = 0x0e | cameraAddr;
  cmd[2] = 0x0d;
  sendCmd(cmd, 6);
  Serial.println("\nCamera initialization done.");
}
/*********************************************************************/
void preCapture()
{
  char cmd[] = { 0xaa, 0x01 | cameraAddr, 0x00, 0x07, 0x00, PIC_FMT };
  unsigned char resp[6];
  while (1)
  {
    clearRxBuf();
    sendCmd(cmd, 6);
    if (readBytes((char *)resp, 6, 100) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x01 && resp[4] == 0 && resp[5] == 0) break;
  }
}
void Capture()
{
  char cmd[] = { 0xaa, 0x06 | cameraAddr, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN >> 8) & 0xff , 0};
  unsigned char resp[6];

  while (1)
  {
    clearRxBuf();
    sendCmd(cmd, 6);
    if (readBytes((char *)resp, 6, 100) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0) break;
  }
  cmd[1] = 0x05 | cameraAddr;
  cmd[2] = 0;
  cmd[3] = 0;
  cmd[4] = 0;
  cmd[5] = 0;
  while (1)
  {
    clearRxBuf();
    sendCmd(cmd, 6);
    if (readBytes((char *)resp, 6, 100) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x05 && resp[4] == 0 && resp[5] == 0) break;
  }
  cmd[1] = 0x04 | cameraAddr;
  cmd[2] = 0x1;
  while (1)
  {
    clearRxBuf();
    sendCmd(cmd, 6);
    if (readBytes((char *)resp, 6, 100) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
    {
      if (readBytes((char *)resp, 6, 1000) != 6)
      {
        continue;
      }
      if (resp[0] == 0xaa && resp[1] == (0x0a | cameraAddr) && resp[2] == 0x01)
      {
        picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
        Serial.print("picTotalLen:");
        Serial.println(picTotalLen);
        break;
      }
    }
  }

}
/*********************************************************************/
char picName[] = "/pic00.jpg";
void GetData()
{
  unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
  if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;

  char cmd[] = { 0xaa, 0x0e | cameraAddr, 0x00, 0x00, 0x00, 0x00 };
  unsigned char pkt[PIC_PKT_LEN];


  picName[4] = picNameNum / 10 + '0';
  picName[5] = picNameNum % 10 + '0';

  if (SD.exists(picName))
  {
    SD.remove(picName);
  }

  myFile = SD.open(picName, FILE_WRITE);
  if (!myFile) {
    //Serial.println("myFile open fail...");
  }
  else {
    for (unsigned int i = 0; i < pktCnt; i++)
    {
      cmd[4] = i & 0xff;
      cmd[5] = (i >> 8) & 0xff;

      int retry_cnt = 0;
retry:
      delay(10);
      clearRxBuf();
      sendCmd(cmd, 6);
      uint16_t cnt = readBytes((char *)pkt, PIC_PKT_LEN, 200);

      unsigned char sum = 0;
      for (int y = 0; y < cnt - 2; y++)
      {
        sum += pkt[y];
      }
      if (sum != pkt[cnt - 2])
      {
        if (++retry_cnt < 100) goto retry;
        else break;
      }

      myFile.write((const uint8_t *)&pkt[4], cnt - 6);
      //if (cnt != PIC_PKT_LEN) break;
    }
    cmd[4] = 0xf0;
    cmd[5] = 0xf0;
    sendCmd(cmd, 6);
  }
  myFile.close();
  Serial.println(picName);
  M5.Lcd.drawJpgFile(SD, picName);
  picNameNum ++;
}
