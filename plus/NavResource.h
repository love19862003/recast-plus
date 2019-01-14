/************************************************
 * \file NavResource.h
 * \date 2019/01/14 16:24
 *
 * \author wufan
 * Contact: love19862003@163.com
 *
 * \brief reader and writer for resource 
 *
 * TODO: long description
 *
 * \note
*************************************************/
#pragma once
#include <string>
#include "NavObject.h"
namespace NavSpace{

  class NavResource{
  public:
    static MeshPtr readMesh(const std::string& file, MeshId id);
    static bool writeMesh(const std::string& file, const Mesh& mesh);
    static ObjectPtr readObject(const std::string& file);
    static bool writeObject(const std::string& file, const MeshObject& obj); 
    static bool readVOF(const std::string& file, VolumeOffCon& volumeOff);
    static bool writeVOF(const std::string& file, const VolumeOffCon& data);
    static ObjectPtr genObject(const std::string& meshFile, const std::string& voloffconn);

  protected:
    static TreeNode*  readNode(std::ifstream& ifile, size_t& maxTri);
    static void writeNode(std::ofstream& ofile, TreeNode* node);
    static bool readBase(std::ifstream& ifile, NavDataBase& data);
    static bool writeBase(std::ofstream& ofile, const NavDataBase& data);
    static bool readVolumeOffConn(std::ifstream& ifile, VolumeOffCon& volumeOff);
    static bool writeVolumeOffConn(std::ofstream& ofile, const VolumeOffCon& data);

    template<typename T>
    static void readPool(std::ifstream& ifile, size_t count, T& pool){
      if (count <= 0){  return; }
      pool.resize(count + 1);
      for (size_t i = 0; i < count; ++i){
        T::ARRAY arr;
        ifile.read((char*)arr.data(), T::ObjectSize());
        pool.add(arr.data(), 1);
      }
    }

    template<typename T>
    static void writePool(std::ofstream& ofile, const T& pool){
      if (pool.count() > 0){
        ofile.write((const char*)pool.pool(), pool.count() * T::ObjectSize());
      }
    }

  protected:
  private:
  };
}
