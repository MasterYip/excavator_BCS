#ifndef _ROS_riki_msgs_Infrared_h
#define _ROS_riki_msgs_Infrared_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Infrared : public ros::Msg
  {
    public:
      typedef float _front_type;
      _front_type front;
      typedef float _rear_type;
      _rear_type rear;

    Infrared():
      front(0),
      rear(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_front;
      u_front.real = this->front;
      *(outbuffer + offset + 0) = (u_front.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front);
      union {
        float real;
        uint32_t base;
      } u_rear;
      u_rear.real = this->rear;
      *(outbuffer + offset + 0) = (u_rear.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_front;
      u_front.base = 0;
      u_front.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->front = u_front.real;
      offset += sizeof(this->front);
      union {
        float real;
        uint32_t base;
      } u_rear;
      u_rear.base = 0;
      u_rear.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rear = u_rear.real;
      offset += sizeof(this->rear);
     return offset;
    }

    const char * getType(){ return "riki_msgs/Infrared"; };
    const char * getMD5(){ return "25f4ca8647150542dc24ea09b80ff4f1"; };

  };

}
#endif