#include <fstream>
#include "NavObject.h"
#include "NavResource.h"
#include "NavMatrix.h"
#include "NavTool.h"
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#ifdef WIN32
#	include <io.h>
#else
#	include <dirent.h>
#	include <cstring>
#endif
enum Type{
  MEGER = 0,
  MESH = 1,
  SCENE = 2,
  NAV = 3,
};




using namespace std;
using namespace NavSpace;

static void scanDirectory(const string& path, const string& ext, vector<string>& filelist);
static void scanDirectoryAppend(const string& path, const string& ext, vector<string>& filelist);


MeshId sid = INVALID_MESH_ID;
MObjId smid = INVALID_MOBJ_ID;
vector<string> fileList;
vector<string> volumeList;

namespace GenSpace{
void genMesh(const string& file, const std::string& out){
  ++sid;
  std::cout << "begin load mesh:" << file << "->" << out << std::endl;
  auto mesh = Mesh::loadMesh(sid, file);
  if (!mesh){
    std::cout << "load mesh:" << file << " with error" << std::endl;
    return;
  }

  if (!mesh->saveMesh(out)){
    std::cout << "save mesh:" << file << " with error" << std::endl;
  }

  std::cout << "save mesh:" << file << "->" << out << std::endl;
}

void genMap(const std::string& file, const std::string& vol, const std::string& out){
  ++sid;
  std::cout << "begin gen map with file:" << file << std::endl;
  auto mesh = Mesh::loadMesh(sid, file);
  if (!mesh){
    std::cout << "load mesh:" << file << " with error" << std::endl;
    return;
  }

  ObjectPtr obj = NavResource::genObject(file, vol);
 
  if (!obj || !obj->saveMap(out)){
    std::cout << "save map:" << file << " with error" << std::endl;
  }

  std::cout << "save map:" << file << (vol.empty() ? "" :  "&& " + vol) << "->" << out << std::endl;
}

void genNav(const std::string& file, const std::string& out){
  std::cout << "begin gen navmesh :" << file << "->" << out << std::endl;
  auto ptr = NavResource::readObject(file);
  if (!ptr){
      std::cout << "load Map:" << file << " with error" << std::endl;
      return;
  }
  static rcContext ctx;
  NavTool tool(&ctx);
  tool.setScenePtr(ptr);
  tool.build();
  tool.saveNavMesh(out);
  std::cout << "save nav:" << file <<  "->" << out << std::endl;
}

void genMeger(const std::string& path, const std::string& name, const std::vector<std::string>& volList, const std::string& out){
  
  std::cout << "begin meger object files:" << path + name << "->" << out << std::endl;
  if (!hasMagicTag(name, MEGER_TAG)){
    std::cout << "genMeger  is not meger tag" << std::endl;
    return;
  }

  std::ifstream ifile(path + MEGER_PATH + name, std::ios::binary | std::ios::in);
  if (!ifile.is_open()){
    std::cout << "genMeger not found file " << (path + MEGER_PATH + name)  << std::endl;
    return;
  }
  MeshId nextMeshId = INVALID_MOBJ_ID;
  static rcContext ctx;
  NavTool tool(&ctx);

  std::string row;
  float pos[3];
  std::string resFile;
  std::string vol;
  while(!ifile.eof()){
    row.clear();
    resFile.clear();
    getline(ifile, row);
    if (row.empty()){ continue;}

    std::istringstream ss(row);
    ss >> pos[0] >> pos[1] >> pos[2] >> resFile ;
    if (resFile.empty()){
      continue;
    }

    if (!hasMagicTag(resFile, OBJ_TAG) && !hasMagicTag(resFile, MESH_TAG)){
      std::cout << "genMeger not found file " << resFile << std::endl;
      assert(false);
      continue;
    }
    vol = setMagicTag(resFile, VOLUMECONN_TAG);
    bool has = std::find(volumeList.begin(), volumeList.end(), vol) != volumeList.end();
    vol = has ? path + VOC_PATH + vol : "";
    WorldItem item;
    item.m_id = ++nextMeshId;
    rcVcopy(item.m_pos.data(), pos);
    auto ptr = NavResource::genObject(path + MEGER_PATH + resFile, vol, item);
    if (!ptr){
      std::cout << "NavResource::genObject error " << path + MEGER_PATH + resFile << std::endl;
      assert(false);
    }
    tool.addObject(ptr);
    
  }
  ifile.close();
  tool.megerObjects(out);
  std::cout << "done meger object files:" << path + name << "->" << out << std::endl;
}



}



int main(int argc,  const char** argv){
  std::string path = "./";
  std::string out;
  std::string in;
  std::string vocPath;
  Type t = NAV;
  
  if (argc > 1){
    path = argv[1];
  }
  if (argc > 2){
    t = static_cast<Type>(stoi(argv[2]));
  }


  if (t == MESH){
    std::cout << "gen object mesh " << std::endl;
    in = path + OBJECT_PATH;
    out = path + MESH_PATH;
    scanDirectory(in, OBJ_TAG, fileList);
  }

  if (t == SCENE){
    std::cout << "gen map mesh " << std::endl;
    in = path + MAP_OBJ_PATH;
    out = path + MAP_PATH;
    vocPath = path + VOC_PATH;
    scanDirectory(in, MESH_TAG, fileList);
    scanDirectory(vocPath, VOLUMECONN_TAG, volumeList);
   
  }

  if (t == NAV){
    std::cout << "gen navmesh " << std::endl;
    in = path + MAP_PATH;
    out = path + NAV_PATH;
    scanDirectory(path + MAP_PATH, MAP_TAG, fileList);
  }

  if (t == MEGER){
    std::cout << "meger objects mesh " << std::endl;
    in = path + MEGER_PATH;
    out = path + MAP_PATH;
    vocPath = path + VOC_PATH;
    scanDirectory(vocPath, VOLUMECONN_TAG, volumeList);
    scanDirectory(in, MEGER_TAG, fileList);
  }

  if (in.empty() || out.empty()){
    assert(false);
    return 1;
  }
 
  for(auto& f : fileList){
    auto file = in + f;
    switch (t){
    case MESH: {
        auto outFile = out + setMagicTag(f, MESH_TAG);
        GenSpace::genMesh(file, outFile);
      }break;
    case SCENE:
      {
        auto vol = setMagicTag(f, VOLUMECONN_TAG);
        bool has = std::find(volumeList.begin(), volumeList.end(), vol) != volumeList.end();
        vol = has ? vocPath + vol : "";
        auto outFile = out + setMagicTag(f, MAP_TAG);
        GenSpace::genMap(file, vol, outFile);
      }
      break;
    case NAV: {
        auto outFile = out + setMagicTag(f, NAVMESH_TAG);
        GenSpace::genNav(file, outFile);
      }break;
    case MEGER :{
        auto outFile = out + setMagicTag(f, MAP_TAG);
        GenSpace::genMeger(path, f, volumeList, outFile);
      }break;
    default:
      break;
    }
  }

  return  0;
}

void scanDirectoryAppend(const string& path, const string& ext, vector<string>& filelist){
#ifdef WIN32
  string pathWithExt = path + "/*" + ext;

  _finddata_t dir;
  intptr_t fh = _findfirst(pathWithExt.c_str(), &dir);
  if (fh == -1L){
    return;
  }

  do{
    filelist.push_back(dir.name);
  } while (_findnext(fh, &dir) == 0);
  _findclose(fh);
#else
  dirent* current = 0;
  DIR* dp = opendir(path.c_str());
  if (!dp){
    return;
  }

  int extLen = strlen(ext.c_str());
  while ((current = readdir(dp)) != 0){
    int len = strlen(current->d_name);
    if (len > extLen && strncmp(current->d_name + len - extLen, ext.c_str(), extLen) == 0){
      filelist.push_back(current->d_name);
    }
  }
  closedir(dp);
#endif

  // Sort the list of files alphabetically.
  std::sort(filelist.begin(), filelist.end());
}

void scanDirectory(const string& path, const string& ext, vector<string>& filelist){
  filelist.clear();
  scanDirectoryAppend(path, ext, filelist);
}