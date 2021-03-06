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
#include "NavCommon.h"
namespace NavSpace{

  static const std::string OBJECT_PATH = "./obj/";
  static const std::string MESH_PATH = "./mesh/";
  static const std::string NAV_PATH = "./nav/";
  static const std::string MAP_PATH = "./map/";
  static const std::string DUMP_PATH = "./dump/";
  static const std::string VOC_PATH = "./voc/";
  static const std::string MERGE_PATH = "./merge/";
  static const std::string MAP_OBJ_PATH = "./map_obj/";

  static const std::string MESH_TAG = ".mesh";
  static const std::string MAP_TAG = ".map";
  static const std::string VOLUMECONN_TAG = ".voc";
  static const std::string OBJ_TAG = ".obj";
  static const std::string DUMP_TAG = ".dump";
  static const std::string NAVMESH_TAG = ".nav";
  static const std::string MERGE_TAG = ".merge";



  inline bool hasMagicTag(const std::string& path, const std::string& tag){
    auto npos = path.find_last_of(".");
    if (npos == std::string::npos || path.substr(npos) != tag){
      return false;
    }
    return true;
  }

  inline std::string setMagicTag(const std::string& file, const std::string& tag){
    auto path = file;
    auto npos = path.find_last_of(".");
    if (npos != std::string::npos){
      if (path.substr(npos) != tag){
        path = path.substr(0, npos) + tag;
      }
    } else{
      path += tag;
    }
    return path;
  }

  class NavResource{
  public:
    static MeshPtr loadObj(const std::string& file, const MeshId id);
    static MeshPtr readMesh(const std::string& file, MeshId id);
    static bool writeMesh(const std::string& file, const Mesh& mesh);
    static ObjectPtr readObject(const std::string& file);
    static bool writeObject(const std::string& file, const MeshObject& obj); 
    static bool readVOF(const std::string& file, VolumeOffCon& volumeOff);
    static bool writeVOF(const std::string& file, const VolumeOffCon& data);
    static ObjectPtr genObject(const std::string& meshFile, const std::string& voloffconn, const WorldItem& item = WorldItem());

  protected:
    static TreeNode*  readNode(std::ifstream& ifile, size_t& maxTri);
    static void writeNode(std::ofstream& ofile, TreeNode* node);
    static bool readBase(std::ifstream& ifile, NavDataBase& data);
    static bool writeBase(std::ofstream& ofile, const NavDataBase& data);
    static bool readVolumeOffConn(std::ifstream& ifile, VolumeOffCon& volumeOff);
    static bool writeVolumeOffConn(std::ofstream& ofile, const VolumeOffCon& data);

  

  protected:
  private:
  };
}
