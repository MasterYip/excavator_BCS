#ifndef _ROS_cartographer_ros_msgs_SubmapTexture_h
#define _ROS_cartographer_ros_msgs_SubmapTexture_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace cartographer_ros_msgs
{

  class SubmapTexture : public ros::Msg
  {
    public:
      uint32_t cells_length;
      typedef uint8_t _cells_type;
      _cells_type st_cells;
      _cells_type * cells;
      typedef int32_t _width_type;
      _width_type width;
      typedef int32_t _height_type;
      _height_type height;
      typedef double _resolution_type;
      _resolution_type resolution;
      typedef geometry_msgs::Pose _slice_pose_type;
      _slice_pose_type slice_pose;

    SubmapTexture():
      cells_length(0), cells(NULL),
      width(0),
      height(0),
      resolution(0),
      slice_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cells_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cells_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cells_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cells_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cells_length);
      for( uint32_t i = 0; i < cells_length; i++){
      *(outbuffer + offset + 0) = (this->cells[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cells[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      union {
        double real;
        uint64_t base;
      } u_resolution;
      u_resolution.real = this->resolution;
      *(outbuffer + offset + 0) = (u_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_resolution.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_resolution.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_resolution.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_resolution.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_resolution.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->resolution);
      offset += this->slice_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t cells_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cells_length);
      if(cells_lengthT > cells_length)
        this->cells = (uint8_t*)realloc(this->cells, cells_lengthT * sizeof(uint8_t));
      cells_length = cells_lengthT;
      for( uint32_t i = 0; i < cells_length; i++){
      this->st_cells =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_cells);
        memcpy( &(this->cells[i]), &(this->st_cells), sizeof(uint8_t));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        double real;
        uint64_t base;
      } u_resolution;
      u_resolution.base = 0;
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->resolution = u_resolution.real;
      offset += sizeof(this->resolution);
      offset += this->slice_pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "cartographer_ros_msgs/SubmapTexture"; };
    const char * getMD5(){ return "26187fc048d2d8e578b6c781f3b53158"; };

  };

}
#endif