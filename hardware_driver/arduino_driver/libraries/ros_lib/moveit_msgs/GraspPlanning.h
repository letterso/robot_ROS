#ifndef _ROS_SERVICE_GraspPlanning_h
#define _ROS_SERVICE_GraspPlanning_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/Grasp.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "moveit_msgs/CollisionObject.h"

namespace moveit_msgs
{

static const char GRASPPLANNING[] = "moveit_msgs/GraspPlanning";

  class GraspPlanningRequest : public ros::Msg
  {
    public:
      const char* group_name;
      moveit_msgs::CollisionObject target;
      uint8_t support_surfaces_length;
      char* st_support_surfaces;
      char* * support_surfaces;
      uint8_t candidate_grasps_length;
      moveit_msgs::Grasp st_candidate_grasps;
      moveit_msgs::Grasp * candidate_grasps;
      uint8_t movable_obstacles_length;
      moveit_msgs::CollisionObject st_movable_obstacles;
      moveit_msgs::CollisionObject * movable_obstacles;

    GraspPlanningRequest():
      group_name(""),
      target(),
      support_surfaces_length(0), support_surfaces(NULL),
      candidate_grasps_length(0), candidate_grasps(NULL),
      movable_obstacles_length(0), movable_obstacles(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_group_name = strlen(this->group_name);
      memcpy(outbuffer + offset, &length_group_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->group_name, length_group_name);
      offset += length_group_name;
      offset += this->target.serialize(outbuffer + offset);
      *(outbuffer + offset++) = support_surfaces_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < support_surfaces_length; i++){
      uint32_t length_support_surfacesi = strlen(this->support_surfaces[i]);
      memcpy(outbuffer + offset, &length_support_surfacesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->support_surfaces[i], length_support_surfacesi);
      offset += length_support_surfacesi;
      }
      *(outbuffer + offset++) = candidate_grasps_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < candidate_grasps_length; i++){
      offset += this->candidate_grasps[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = movable_obstacles_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < movable_obstacles_length; i++){
      offset += this->movable_obstacles[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_group_name;
      memcpy(&length_group_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_group_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_group_name-1]=0;
      this->group_name = (char *)(inbuffer + offset-1);
      offset += length_group_name;
      offset += this->target.deserialize(inbuffer + offset);
      uint8_t support_surfaces_lengthT = *(inbuffer + offset++);
      if(support_surfaces_lengthT > support_surfaces_length)
        this->support_surfaces = (char**)realloc(this->support_surfaces, support_surfaces_lengthT * sizeof(char*));
      offset += 3;
      support_surfaces_length = support_surfaces_lengthT;
      for( uint8_t i = 0; i < support_surfaces_length; i++){
      uint32_t length_st_support_surfaces;
      memcpy(&length_st_support_surfaces, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_support_surfaces; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_support_surfaces-1]=0;
      this->st_support_surfaces = (char *)(inbuffer + offset-1);
      offset += length_st_support_surfaces;
        memcpy( &(this->support_surfaces[i]), &(this->st_support_surfaces), sizeof(char*));
      }
      uint8_t candidate_grasps_lengthT = *(inbuffer + offset++);
      if(candidate_grasps_lengthT > candidate_grasps_length)
        this->candidate_grasps = (moveit_msgs::Grasp*)realloc(this->candidate_grasps, candidate_grasps_lengthT * sizeof(moveit_msgs::Grasp));
      offset += 3;
      candidate_grasps_length = candidate_grasps_lengthT;
      for( uint8_t i = 0; i < candidate_grasps_length; i++){
      offset += this->st_candidate_grasps.deserialize(inbuffer + offset);
        memcpy( &(this->candidate_grasps[i]), &(this->st_candidate_grasps), sizeof(moveit_msgs::Grasp));
      }
      uint8_t movable_obstacles_lengthT = *(inbuffer + offset++);
      if(movable_obstacles_lengthT > movable_obstacles_length)
        this->movable_obstacles = (moveit_msgs::CollisionObject*)realloc(this->movable_obstacles, movable_obstacles_lengthT * sizeof(moveit_msgs::CollisionObject));
      offset += 3;
      movable_obstacles_length = movable_obstacles_lengthT;
      for( uint8_t i = 0; i < movable_obstacles_length; i++){
      offset += this->st_movable_obstacles.deserialize(inbuffer + offset);
        memcpy( &(this->movable_obstacles[i]), &(this->st_movable_obstacles), sizeof(moveit_msgs::CollisionObject));
      }
     return offset;
    }

    const char * getType(){ return GRASPPLANNING; };
    const char * getMD5(){ return "c234e9a645708cc86b57a43999746ae6"; };

  };

  class GraspPlanningResponse : public ros::Msg
  {
    public:
      uint8_t grasps_length;
      moveit_msgs::Grasp st_grasps;
      moveit_msgs::Grasp * grasps;
      moveit_msgs::MoveItErrorCodes error_code;

    GraspPlanningResponse():
      grasps_length(0), grasps(NULL),
      error_code()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = grasps_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < grasps_length; i++){
      offset += this->grasps[i].serialize(outbuffer + offset);
      }
      offset += this->error_code.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t grasps_lengthT = *(inbuffer + offset++);
      if(grasps_lengthT > grasps_length)
        this->grasps = (moveit_msgs::Grasp*)realloc(this->grasps, grasps_lengthT * sizeof(moveit_msgs::Grasp));
      offset += 3;
      grasps_length = grasps_lengthT;
      for( uint8_t i = 0; i < grasps_length; i++){
      offset += this->st_grasps.deserialize(inbuffer + offset);
        memcpy( &(this->grasps[i]), &(this->st_grasps), sizeof(moveit_msgs::Grasp));
      }
      offset += this->error_code.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GRASPPLANNING; };
    const char * getMD5(){ return "bb8169d403b6e9f96bf61e22a50d13ae"; };

  };

  class GraspPlanning {
    public:
    typedef GraspPlanningRequest Request;
    typedef GraspPlanningResponse Response;
  };

}
#endif
