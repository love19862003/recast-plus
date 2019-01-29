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
#include <array>
namespace NavSpace{
  typedef unsigned int MObjId;
  typedef unsigned int MeshId;
  typedef std::vector<MObjId> MObjList;
  typedef std::function<void(const MObjList&, const MObjList&)> NotifyCall;
  typedef std::array<float, 3> NavPos;

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
    bool  filterLowHangingObstacles;
    bool  filterLedgeSpans;
    bool  filterWalkableLowHeightSpans;
    
    float tileSize;
    float navBmin[3];
    float navBmax[3];

    NavSetting(){
      cellSize = 0.4f;
      cellHeight = 0.2f;
      agentHeight = 2.0f;
      agentRadius = 0.6f;
      agentMaxClimb = 0.9f;
      agentMaxSlope = 45.0f;
      regionMinSize = 8;
      regionMergeSize = 20;
      edgeMaxLen = 12.0f;
      edgeMaxError = 1.3f;
      vertsPerPoly = 6.0f;
      detailSampleDist = 6.0f;
      detailSampleMaxError = 1.0f;
      partitionType = SAMPLE_PARTITION_WATERSHED;
      filterLowHangingObstacles = true;
      filterLedgeSpans = true;
      filterWalkableLowHeightSpans = true;
      tileSize = 124;
      navBmin[0] = navBmin[1] = navBmin[2] = 0.f;
      navBmax[0] = navBmax[1] = navBmax[2] = 0.f;
    }
  };

  class NonCopyAble{
  protected:
    NonCopyAble() = default;
    ~NonCopyAble() = default;

  private:
    NonCopyAble(const NonCopyAble&) = delete;
    NonCopyAble& operator=(const NonCopyAble&) = delete;
  };

  struct WorldItem{

    WorldItem(){
      m_pos.fill(0.f);
      m_o = 0.f;
      m_scale = 1.f;
      m_mesh = INVALID_MESH_ID;
      m_id = INVALID_MOBJ_ID;
    }

    NavPos m_pos;
    float m_o;
    float m_scale;
    MeshId m_mesh;
    MObjId m_id;
  };


  class Mesh;
  typedef std::shared_ptr<Mesh> MeshPtr;
  class MeshObject;
  typedef std::shared_ptr<MeshObject> ObjectPtr;
  struct VolumeOffCon;
  struct TreeNode;
  struct NavDataBase;



}

