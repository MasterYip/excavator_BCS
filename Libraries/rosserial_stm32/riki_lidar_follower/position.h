#ifndef _ROS_riki_lidar_follower_position_h
#define _ROS_riki_lidar_follower_position_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_lidar_follower
{

  class position : public ros::Msg
  {
    public:
      typedef float _angleX_type;
      _angleX_type angleX;
      typedef float _angleY_type;
      _angleY_type angleY;
      typedef float _distance_type;
      _distance_type distance;

    position():
      angleX(0),
      angleY(0),
      distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angleX;
      u_angleX.real = this->angleX;
      *(outbuffer + offset + 0) = (u_angleX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angleX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angleX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angleX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angleX);
      union {
        float real;
        uint32_t base;
      } u_angleY;
      u_angleY.real = this->angleY;
      *(outbuffer + offset + 0) = (u_angleY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angleY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angleY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angleY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angleY);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angleX;
      u_angleX.base = 0;
      u_angleX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angleX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angleX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angleX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angleX = u_angleX.real;
      offset += sizeof(this->angleX);
      union {
        float real;
        uint32_t base;
      } u_angleY;
      u_angleY.base = 0;
      u_angleY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angleY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angleY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angleY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angleY = u_angleY.real;
      offset += sizeof(this->angleY);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
     return offset;
    }

    const char * getType(){ return "riki_lidar_follower/position"; };
    const char * getMD5(){ return "e4df5e09e92d9d2b4758c9aab7a9ebeb"; };

  };

}
#endif