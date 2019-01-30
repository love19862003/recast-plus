/************************************************
 * \file NavResource.cpp
 * \date 2019/01/14 16:25
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
#include <fstream>
#include <sstream>
#include "NavObject.h"
#include "NavResource.h"

namespace NavSpace{
  static const unsigned int MESH_MAGIC = '1' << 24 | '0' << 16 | '0' << 8 | '0';
  static const unsigned int MAP_MAGIC = '9' << 24 | '0' << 16 | '0' << 8 | '0';
  static const unsigned int VOLUME_OFF_MAGIC = '1' << 24 | '0' << 16 | '0' << 8 | '0';
#pragma pack(push, 1)
  struct DataBaseHeader{
    unsigned int version;
    unsigned int vers;
    unsigned int tris;
  };

  struct DataMapHeader{
    unsigned int version;
    unsigned int tree;
  };

  struct DataVolumeConnHeader{
    unsigned int version;
    unsigned int volmes;
    unsigned int offcons;
  };
#pragma pack(pop)

  template<typename T>
  static void readPool(std::ifstream& ifile, size_t count, T& pool){
    if (count <= 0){ return; }
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

  bool NavResource::readVOF(const std::string& file, VolumeOffCon& volumeOff){
  
    if (!hasMagicTag(file, VOLUMECONN_TAG)){
      return false;
    }

    try{
      std::ifstream ifile(file, std::ios::in | std::ios::binary);
      if (!ifile.is_open()){
        return false;
      }
      if (!readVolumeOffConn(ifile, volumeOff)){
        return nullptr;
      }

      ifile.close();
      return true;

    } catch (...){
      return false;
    }

  }
  bool NavResource::writeVOF(const std::string& file, const VolumeOffCon& data){
    auto path = setMagicTag(file, VOLUMECONN_TAG);
    try{
      std::ofstream ofile(path, std::ios::out | std::ios::binary | std::ios::trunc);
      writeVolumeOffConn(ofile, data);
      ofile.close();
      return true;
    } catch (...){
      return false;
    }
  }

  static bool isVertRow(const std::string& row){
    return row.size() > 2 &&  row[0] == 'v' && row[1] != 'n' && row[1] != 't';
  }

  static bool isTriRow(const std::string& row){
    return !row.empty() && row[0] == 'f';
  } 

  MeshPtr NavResource::loadObj(const std::string& file, const MeshId id){
    if (!hasMagicTag(file, OBJ_TAG)) return nullptr;
    auto mesh = std::make_shared<Mesh>(file, id);
    if (!mesh) return nullptr;
    std::ifstream ifile(file, std::ios::binary | std::ios::in);
    if (!ifile.is_open()) return nullptr;

    std::string row;
    float pos[3] = {0.f, 0.f,0.f};
    std::vector<int> tri;
    tri.reserve(64);

    auto readTri = [](const std::string& str, size_t max){
      int r = std::stoi(str);
      r = r < 0 ? r + max : r - 1;
      assert(r < max && r >= 0);
      return r;
    };

    while(getline(ifile, row)){
      if (row.empty() ||  row[0] == '#'){ continue;}
      if (isVertRow(row)){
        std::istringstream ss(row.substr(1));
        ss >> pos[0] >> pos[1] >> pos[2];
        mesh->m_verts.add(pos);
      }else if (isTriRow(row)){
        std::istringstream ss(row.substr(1));
        const size_t nvert = mesh->m_verts.count();
        std::string str;
        tri.clear();
        do 
        {
          str.clear();
          ss >> str;
          if (str.empty()){ break;}
          int t = readTri(str, nvert);
          assert(t < nvert);
          tri.push_back(t);
        } while ( !str.empty());

        for (size_t i = 2; i < tri.size(); ++i){
          int t[3] = {tri[0], tri[i-1], tri[i]};
          mesh->m_tris.add(t);
        }
      }
    }

    ifile.close();
    mesh->calcuteBouns();
    return mesh;
  }

  MeshPtr NavResource::readMesh(const std::string& file, MeshId id){
    if (!hasMagicTag(file, MESH_TAG)){
      return nullptr;
    }
 
    auto res = std::make_shared<Mesh>(file, id);
    if (!res) return nullptr;
    try{
      std::ifstream ifile(file, std::ios::in | std::ios::binary);
      if (!ifile.is_open()){
        return nullptr;
      }
      if (!readBase(ifile, *res)){
        return nullptr;
      }

      ifile.close();
      return res;

    } catch (...){
      return nullptr;
    }
  }
  bool NavResource::writeMesh(const std::string& file, const Mesh& mesh){
    auto path = setMagicTag(file, MESH_TAG);
    
    try{
        std::ofstream ofile(path, std::ios::out | std::ios::binary | std::ios::trunc);
        writeBase(ofile, mesh);
        ofile.close();
        return true;
      } catch (...){
        return false;
      }
    }
  ObjectPtr NavResource::readObject(const std::string& file){

    if (!hasMagicTag(file, MAP_TAG)){
      return nullptr;
    }

    ObjectPtr res(new MeshObject(INVALID_MOBJ_ID));
    try{
      std::ifstream ifile(file, std::ios::in | std::ios::binary);
      if (!ifile.is_open()){
        return nullptr;
      }
      if (!readBase(ifile, *res)){
        return nullptr;
      }

      DataMapHeader header;
      ifile.read((char*)&header, sizeof(DataMapHeader));
      if (!readVolumeOffConn(ifile, res->m_volumeOffConn)){ return nullptr; }
      if (header.tree){ res->m_tree = readNode(ifile, res->m_maxTriPerChunk); }
      res->calculateNormals();
      ifile.close();
      return res;
    } catch (...){
      return nullptr;
    }
  }

  ObjectPtr NavResource::genObject(const std::string& meshFile, const std::string& voloffconn, const WorldItem& item){
    auto mesh = Mesh::loadMesh(INVALID_MESH_ID, meshFile);
    if (!mesh){
      return nullptr;
    }

    ObjectPtr res(new MeshObject(INVALID_MOBJ_ID));
    if (!res){
      return nullptr;
    }
    res->m_item = item;
    if((!voloffconn.empty() && !readVOF(voloffconn, res->m_volumeOffConn)) || !res->initFromMesh(mesh)){
      return nullptr;
    }
    return res;
  }
  bool NavResource::readVolumeOffConn(std::ifstream& ifile, VolumeOffCon& volumeOff){
    DataVolumeConnHeader header;
    ifile.read((char*)&header, sizeof(DataVolumeConnHeader));
    if (header.version != VOLUME_OFF_MAGIC){
      return false;
    }
    readPool(ifile, header.offcons, volumeOff.m_offCons);
    readPool(ifile, header.volmes, volumeOff.m_volumes);
    return true;
  }
  bool NavResource::writeObject(const std::string& file, const MeshObject& obj){
    auto path = setMagicTag(file, MAP_TAG);
    try{
      std::ofstream ofile(path, std::ios::out | std::ios::binary | std::ios::trunc);
      writeBase(ofile, obj);
      DataMapHeader header;
      header.version = MAP_MAGIC;
      header.tree = obj.m_tree ? 1 : 0;
      ofile.write((const char*)&header, sizeof(DataMapHeader));
      writeVolumeOffConn(ofile, obj.m_volumeOffConn);
      if (header.tree > 0){
        writeNode(ofile, obj.m_tree);
      }

      ofile.close();
      return true;
    } catch (...){
      return false;
    }
  }

 
  bool NavResource::writeVolumeOffConn(std::ofstream& ofile, const VolumeOffCon& data){
    DataVolumeConnHeader header;
    header.version = VOLUME_OFF_MAGIC;
    header.offcons = data.m_offCons.count();
    header.volmes = data.m_volumes.count();
    ofile.write((const char*)&header, sizeof(DataVolumeConnHeader));
    writePool(ofile, data.m_offCons);
    writePool(ofile, data.m_volumes);
    return true;
  }
  TreeNode*  NavResource::readNode(std::ifstream& ifile, size_t& maxTri){
    TreeNode* node = new TreeNode;
#ifdef TREE_DEBUG
    ifile.read((char*)&node->depth, sizeof(int));
    ifile.read((char*)&node->num, sizeof(int));
#endif
    ifile.read((char*)&node->leaf, sizeof(bool));
    ifile.read((char*)node->bouns.bmin.data(), sizeof(TreeBouns::Bouns::value_type) * node->bouns.bmin.size());
    ifile.read((char*)node->bouns.bmax.data(), sizeof(TreeBouns::Bouns::value_type) * node->bouns.bmax.size());
    size_t c = 0;
    ifile.read((char*)&c, sizeof(size_t));
    if (c > 0){
      readPool(ifile, c, node->tris);
    }

    if (c > maxTri){ maxTri = c; }

    for (size_t i = 0; i < treeChildCount(); ++i){
      if (node->leaf){
        node->child[i] = nullptr;
      } else{
        node->child[i] = readNode(ifile, maxTri);
      }
    }
    return node;
  }
  void NavResource::writeNode(std::ofstream& ofile, TreeNode* node){
    if (!node) return;
#ifdef TREE_DEBUG
    ofile.write((const char*)&node->depth, sizeof(int));
    ofile.write((const char*)&node->num, sizeof(int));
#endif
    ofile.write((const char*)&node->leaf, sizeof(bool));
    ofile.write((const char*)node->bouns.bmin.data(), sizeof(TreeBouns::Bouns::value_type) * node->bouns.bmin.size());
    ofile.write((const char*)node->bouns.bmax.data(), sizeof(TreeBouns::Bouns::value_type) * node->bouns.bmax.size());
    size_t c = node->tris.count();
    ofile.write((const char*)&c, sizeof(size_t));
    if (c > 0){
      writePool(ofile, node->tris);
    }
    if (!node->leaf){
      for (size_t i = 0; i < treeChildCount(); ++i){
        writeNode(ofile, node->child[i]);
      }
    }
  }
  bool NavResource::readBase(std::ifstream& ifile, NavDataBase& data){
    DataBaseHeader head;
    ifile.read((char*)&head, sizeof(DataBaseHeader));
    if (head.version != MESH_MAGIC){ return false; }
    readPool(ifile, head.vers, data.m_verts);
    readPool(ifile, head.tris, data.m_tris);
#ifdef _DEBUG
    data.m_tris.call([&data](const int*tri, size_t){
      assert(data.m_verts.count() >  static_cast<size_t>(tri[0]));
      assert(data.m_verts.count() >  static_cast<size_t>(tri[1]));
      assert(data.m_verts.count() >  static_cast<size_t>(tri[2]));
    });
#endif // _DEBUG
    data.calcuteBouns();
    return true;
  }
  bool NavResource::writeBase(std::ofstream& ofile, const NavDataBase& data){
    DataBaseHeader head;
    head.version = MESH_MAGIC;
    head.vers = data.m_verts.count();
    head.tris = data.m_tris.count();
    ofile.write((const char*)&head, sizeof(DataBaseHeader));
    writePool(ofile, data.m_verts);
    writePool(ofile, data.m_tris);
    return true;
  }
}