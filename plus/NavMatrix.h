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

  struct Matirx{
    static void matirx(float* out, const float in[3], const WorldItem& w);
  };
  

}
