// AssimpLoader.h


#ifndef _ASSIMPLOADER_H_
#define _ASSIMPLOADER_H_


#include <iostream>
#include <memory>

namespace cmod {
      class Material;
      class Mesh;
      class Model;
      class TextureLoader;
}
namespace Assimp {
      class Importer;
}

#include "assimp/mesh.h"
#include "assimp/scene.h"
#include "assimp/types.h"



// Doc: http://assimp.sourceforge.net/lib_html/index.html


class AssimpLoader
{
public:

      AssimpLoader();
      virtual ~AssimpLoader();

      bool isExtensionSupported(const std::string& extension) const;

      cmod::Model* loadModel(const std::string& filename, cmod::TextureLoader* textureLoader = NULL);

protected:

      void processNode(const aiNode &node, const aiMatrix4x4 &parentTransform, const aiScene &scene, cmod::Model *model, cmod::TextureLoader* textureLoader);

      static cmod::Material* convertAssimpMaterial(const aiMaterial &assimpMaterial, cmod::TextureLoader* textureLoader);
      static cmod::Mesh *convertAssimpMesh(const aiMesh &assimpMesh, const aiMatrix4x4 &nodeTransform);

private:

      std::unique_ptr<Assimp::Importer> _importer;
};


#endif // ASSIMPLOADER_H_
