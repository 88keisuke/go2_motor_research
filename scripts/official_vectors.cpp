// Offline only: official SDK serializer/decoder, never constructs SerialPort.
#include <iostream>
#include <iomanip>
#include <cstring>
#include "unitreeMotor/unitreeMotor.h"
int main() {
  std::cout << "gear=" << queryGearRatio(MotorType::GO_M8010_6) << "\n";
  const float values[][5] = {{0,0,0,0,0}, {1,-2,3,0.5,0.05}, {-0.1,0.1,-0.1,0.1,0.01}, {6.2832,-6.2832,1,1,1}};
  for (auto &v:values) {
    MotorCmd cmd{}; cmd.motorType=MotorType::GO_M8010_6; cmd.id=2; cmd.mode=1;
    cmd.q=v[0]; cmd.dq=v[1]; cmd.tau=v[2]; cmd.kp=v[3]; cmd.kd=v[4];
    // SDK constructor leaves the packed buffer uninitialized; explicitly set
    // reserved bits to zero for deterministic vectors.
    std::memset(cmd.get_motor_send_data(),0,17);
    cmd.modify_data(&cmd);
    for(int i=0;i<17;i++) std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(cmd.get_motor_send_data()[i])<<(i==16?'\n':' ');
  }
  // CRC bytes are supplied independently by the official crc_ccitt header.
  MotorData data{}; data.motorType=MotorType::GO_M8010_6;
  const uint8_t raw[]={0xfd,0xee,0x12,0x00,0xff,0xf8,0xff,0x38,0x24,0,0,0x1c,0x05,0x6c,0,0};
  std::memcpy(data.get_motor_recv_data(),raw,16);
  // Defined in the SDK header, independent of Python implementation.
  uint8_t* p=data.get_motor_recv_data();
  extern uint16_t crc_for_vector(const uint8_t*);
  uint16_t crc=crc_for_vector(p); p[14]=crc&255; p[15]=crc>>8;
  bool ok=data.extract_data(&data);
  std::cout<<std::dec<<std::setprecision(10)<<"rx="<<ok<<",id="<<unsigned(data.motor_id)<<",mode="<<unsigned(data.mode)<<",q="<<data.q<<",dq="<<data.dq<<",tau="<<data.tau<<",temp="<<data.temp<<",error="<<data.merror<<",aux_sdk="<<data.footForce<<"\n";
  for(int i=0;i<16;i++) std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(p[i])<<(i==15?'\n':' ');
}
#include "crc/crc_ccitt.h"
uint16_t crc_for_vector(const uint8_t* p) {return crc_ccitt(0,p,14);}
