#include "NavObject.h"
#include "NavResource.h"
#include "NavMatrix.h"
#include "NavTool.h"
#include <vector>
#include <string>
#include <iostream>
#ifdef WIN32
#	include <io.h>
#else
#	include <dirent.h>
#	include <cstring>
#endif
enum Type{
  MESH = 0,
  SCENE = 1,
  NAV = 2,
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
    in = path + OBJECT_PATH;
    out = path + MESH_PATH;
    scanDirectory(in, OBJ_TAG, fileList);
   
  }

  if (t == SCENE){
    in = path + MESH_PATH;
    out = path + MAP_PATH;
    vocPath = path + VOC_PATH;
    scanDirectory(in, MESH_TAG, fileList);
    scanDirectoryAppend(vocPath, VOLUMECONN_TAG, volumeList);
   
  }

  if (t == NAV){
    in = path + MAP_PATH;
    out = path + NAV_PATH;
    scanDirectory(path + MAP_PATH, MAP_TAG, fileList);
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