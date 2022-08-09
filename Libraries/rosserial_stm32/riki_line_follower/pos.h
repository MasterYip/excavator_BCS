#ifndef _ROS_riki_line_follower_pos_h
#define _ROS_riki_line_follower_pos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_line_follower
{

  class pos : public ros::Msg
  {
    public:
      typedef int32_t _direction_type;
      _direction_type direction;

    pos():
      direction(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_direction;
      u_direction.real = this->direction;
      *(outbuffer + offset + 0) = (u_direction.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_direction.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_direction.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_direction.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->direction);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_direction;
      u_direction.base = 0;
      u_direction.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_direction.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_direction.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_direction.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->direction = u_direction.real;
      offset += sizeof(this->direction);
     return offset;
    }

    const char * getType(){ return "riki_line_follower/pos"; };
    const char * getMD5(){ return "3b685cbe3a2d37c79069d18fc947a4de"; };

  };

}
#endif