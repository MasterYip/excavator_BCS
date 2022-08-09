#ifndef _ROS_cartographer_ros_msgs_LandmarkList_h
#define _ROS_cartographer_ros_msgs_LandmarkList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "cartographer_ros_msgs/LandmarkEntry.h"

namespace cartographer_ros_msgs
{

  class LandmarkList : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t landmark_length;
      typedef cartographer_ros_msgs::LandmarkEntry _landmark_type;
      _landmark_type st_landmark;
      _landmark_type * landmark;

    LandmarkList():
      header(),
      landmark_length(0), landmark(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->landmark_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->landmark_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->landmark_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->landmark_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->landmark_length);
      for( uint32_t i = 0; i < landmark_length; i++){
      offset += this->landmark[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t landmark_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      landmark_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      landmark_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      landmark_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->landmark_length);
      if(landmark_lengthT > landmark_length)
        this->landmark = (cartographer_ros_msgs::LandmarkEntry*)realloc(this->landmark, landmark_lengthT * sizeof(cartographer_ros_msgs::LandmarkEntry));
      landmark_length = landmark_lengthT;
      for( uint32_t i = 0; i < landmark_length; i++){
      offset += this->st_landmark.deserialize(inbuffer + offset);
        memcpy( &(this->landmark[i]), &(this->st_landmark), sizeof(cartographer_ros_msgs::LandmarkEntry));
      }
     return offset;
    }

    const char * getType(){ return "cartographer_ros_msgs/LandmarkList"; };
    const char * getMD5(){ return "301d0343edf9ac469d5bbb0142518bf8"; };

  };

}
#endif