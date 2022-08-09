#ifndef _ROS_riki_msgs_DHT22_h
#define _ROS_riki_msgs_DHT22_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class DHT22 : public ros::Msg
  {
    public:
      typedef float _Temperature_type;
      _Temperature_type Temperature;
      typedef float _Humidity_type;
      _Humidity_type Humidity;

    DHT22():
      Temperature(0),
      Humidity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_Temperature;
      u_Temperature.real = this->Temperature;
      *(outbuffer + offset + 0) = (u_Temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Temperature);
      union {
        float real;
        uint32_t base;
      } u_Humidity;
      u_Humidity.real = this->Humidity;
      *(outbuffer + offset + 0) = (u_Humidity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Humidity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Humidity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Humidity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Humidity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_Temperature;
      u_Temperature.base = 0;
      u_Temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Temperature = u_Temperature.real;
      offset += sizeof(this->Temperature);
      union {
        float real;
        uint32_t base;
      } u_Humidity;
      u_Humidity.base = 0;
      u_Humidity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Humidity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Humidity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Humidity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Humidity = u_Humidity.real;
      offset += sizeof(this->Humidity);
     return offset;
    }

    const char * getType(){ return "riki_msgs/DHT22"; };
    const char * getMD5(){ return "5444ed9cc78ceaace5058c55fe815905"; };

  };

}
#endif