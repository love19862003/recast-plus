/************************************************
 * \file NavMatrix.h
 * \date 2019/01/14 16:16
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
#include <array>
#include "NavCommon.h"
namespace NavSpace{
  struct WorldPos{

    WorldPos(){
      m_pos.fill(0.f);
      m_o = 0.f;
      m_scale = 1.f;
      m_mesh = INVALID_MESH_ID;
    }

    std::array<float, 3> m_pos;
    float m_o;
    float m_scale;
    MeshId m_mesh;
  };

  struct Matirx{
    static void matirx(float* out, const float in[3], const WorldPos& w);
  };
  

}
