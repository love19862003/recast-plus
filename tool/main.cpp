#include "NavObject.h"
#include "NavResource.h"
#include "NavMatrix.h"
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
  auto mesh = Mesh::loadMesh(sid, file);
  if (!mesh){
    std::cout << "load mesh:" << file << " with error" << std::endl;
    return;
  }

  if (!mesh->saveMesh(out)){
    std::cout << "save mesh:" << file << " with error" << std::endl;
  }
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
}

void genNav(const string& file, const std::string& out){

}
}


int main(int argc,  const char** argv){
  std::string path = "./obj/";
  std::string out = "./mesh/";
  Type t = SCENE;
  if (t == SCENE){
    out = "./map/";
  }


  if (argc > 1){
    path = argv[1];
  }

  if (argc > 2){
    out = argv[2];
  }
  if (argc > 3){
    t = static_cast<Type>(stoi(argv[3]));
  }

  scanDirectory(path, ".obj", fileList);
  if (t == SCENE){
    scanDirectoryAppend(path, ".mesh", fileList);
    scanDirectoryAppend(path, ".voc", volumeList);
  }

  if (t == NAV){
    scanDirectory(path, ".map", fileList);
  }
 
  for(auto& f : fileList){
    auto file = path + f;
    auto outFile = out + f;
    switch (t){
    case MESH:
      GenSpace::genMesh(file, outFile);
      break;
    case SCENE:
      {
        auto ext = f.find_last_of(".");
        auto vol = file.substr(0, ext - 1) + ".voc";
        if (std::find(volumeList.begin(), volumeList.end(), vol) == volumeList.end()){
          vol = "";
        }else{
          vol = path + vol;
        }
        GenSpace::genMap(file, vol, outFile);
      }
      break;
    case NAV:
      GenSpace::genNav(file, outFile);
      break;
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