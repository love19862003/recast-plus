/************************************************
 * \file NavCommon.h
 * \date 2019/01/10 17:17
 *
 * \author wufan
 * Contact: love19862003@163.com
 *
 * \brief 
 *
 * TODO: long description
 *
 * \note
*************************************************/
#pragma once
#include <functional>
#include <vector>
namespace NavSpace{
  typedef unsigned int MObjId;
  typedef unsigned int MeshId;
  typedef std::vector<MObjId> MObjList;
  typedef std::function<void(const MObjList&, const MObjList&)> NotifyCall;

  enum {
    INVALID_MOBJ_ID = 0,
    INVALID_MESH_ID = 0,
  };

  enum NavPartitionType{
    SAMPLE_PARTITION_WATERSHED,
    SAMPLE_PARTITION_MONOTONE,
    SAMPLE_PARTITION_LAYERS,
  };

  /// These are just sample areas to use consistent values across the samples.
  /// The use should specify these base on his needs.
  enum NavPolyAreas{
    SAMPLE_POLYAREA_GROUND,
    SAMPLE_POLYAREA_WATER,
    SAMPLE_POLYAREA_ROAD,
    SAMPLE_POLYAREA_DOOR,
    SAMPLE_POLYAREA_GRASS,
    SAMPLE_POLYAREA_JUMP,
  };
  enum NavPolyFlags{
    SAMPLE_POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
    SAMPLE_POLYFLAGS_SWIM = 0x02,		// Ability to swim (water).
    SAMPLE_POLYFLAGS_DOOR = 0x04,		// Ability to move through doors.
    SAMPLE_POLYFLAGS_JUMP = 0x08,		// Ability to jump.
    SAMPLE_POLYFLAGS_DISABLED = 0x10,		// Disabled polygon
    SAMPLE_POLYFLAGS_ALL = 0xffff	// All abilities.
  };


  struct NavSetting {
    float cellSize;
    float cellHeight;
    float agentHeight;
    float agentRadius;
    float agentMaxClimb;
    float agentMaxSlope;
    float regionMinSize;
    float regionMergeSize;
    float edgeMaxLen;
    float edgeMaxError;
    float vertsPerPoly;
    float detailSampleDist;
    float detailSampleMaxError;
    int   partitionType;
    bool filterLowHangingObstacles;
    bool filterLedgeSpans;
    bool filterWalkableLowHeightSpans;
    
    float tileSize;
    float navBmin[3];
    float navBmax[3];
  };

  class NonCopyAble{
  protected:
    NonCopyAble() = default;
    ~NonCopyAble() = default;

  private:
    NonCopyAble(const NonCopyAble&) = delete;
    NonCopyAble& operator=(const NonCopyAble&) = delete;
  };

}

