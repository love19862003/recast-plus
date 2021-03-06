/************************************************
 * \file NavMatrix.cpp
 * \date 2019/01/14 16:18
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

#include "NavMatrix.h"
#ifdef MATIRX_TYPE_BOOST
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/operation.hpp>
#endif
namespace NavSpace{
#ifdef MATIRX_TYPE_BOOST
  static void smatirx(float* out, const float in[3], const WorldItem& w){
    namespace ublas = boost::numeric::ublas;
    ublas::vector<float> vec(4);
    ublas::matrix<float> m(4, 4, 0.f);
    vec(0) = in[0];
    vec(1) = in[1];
    vec(2) = in[2];
    vec(3) = 1.f;

    const static float pi = 3.14159265358f;
    float o = w.m_o / 180.f * pi;
    m(0, 0) = std::cos(o);
    m(0, 2) = -std::sin(o);
    m(1, 1) = 1.f;

    m(2, 0) = std::sin(o);
    m(2, 2) = std::cos(o);
    m(3, 3) = 1.f;

    ublas::vector<float> res(4, 0.f);
    ublas::axpy_prod(vec, m, res);

    out[0] = w.m_pos[0] + res(0) * w.m_scale;
    out[1] = w.m_pos[1] + res(1) * w.m_scale;
    out[2] = w.m_pos[2] + res(2) * w.m_scale;
  }
#else
  static void smatirx(float* out, const float in[3], const WorldItem& w){
    const static float pi = 3.14159265358f;
    float o = w.m_o / 180.f * pi;
    float c = std::cos(o);
    float s = std::sin(o);
    out[0] = in[0] * c - in[2] * s;
    out[1] = in[1];
    out[2] = in[2] * c + in[0] * s;

    out[0] = w.m_pos[0] + out[0] * w.m_scale;
    out[1] = w.m_pos[1] + out[1] * w.m_scale;
    out[2] = w.m_pos[2] + out[2] * w.m_scale;
  }
#endif
  void Matirx::matirx(float* out, const float in[3], const WorldItem& w){
    smatirx(out, in, w);
  }
}