// AssimpLoader.cpp

#include "AssimpLoader.h"

#include "celmodel/material.h"
#include "celmodel/modelfile.h"

// #include <celutil/debug.h>
#include "assimp/Importer.hpp"
#include "assimp/cimport.h"
#include "assimp/postprocess.h"

#include <assimp/DefaultLogger.hpp>
#include <celmath/mathlib.h>
#include <iostream>
#include <list>
#include <stdexcept>

AssimpLoader::AssimpLoader() : _importer(new Assimp::Importer)
{
    // Affecte GenSmoothNormals : 80 est un bon compromis
    _importer->SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 80.0f);
}

AssimpLoader::~AssimpLoader()
{
}

bool
AssimpLoader::isExtensionSupported(const std::string &extension) const
{
    return _importer->IsExtensionSupported(extension);
}

cmod::Model *
AssimpLoader::loadModel(const std::string &filename, cmod::TextureLoader *textureLoader)
{
    // Create a logger instance
    Assimp::DefaultLogger::create("", Assimp::Logger::NORMAL, aiDefaultLogStream_STDERR);

    // Create a model
    cmod::Model *model = new cmod::Model();

    try
    {
        // the importer object keeps ownership of the data and will * destroy it upon destruction.
        // const aiScene* scene = _importer->ReadFile(filename.c_str(),
        // aiProcessPreset_TargetRealtime_MaxQuality | aiProcess_FlipUVs);
        const aiScene *scene = _importer->ReadFile(filename.c_str(), 0);
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            std::cerr << "Error parsing '" << filename.c_str()
                      << "': " << _importer->GetErrorString() << std::endl;
            delete model;
            return nullptr;
        }

        /*
        ** Certains fichiers ne s'affichent pas correctement (CIC-SAT.3ds)
        ** Un blame de ce fichier :
        ** https://github.com/assimp/assimp/blame/8326535445536bfb0d2d99af922b940f7bfdd6f7/code/3DSConverter.cpp
        ** montre que deux commits ont résolu puis rétabli ce qui nous pose problème aujourd'hui :
        ** https://github.com/assimp/assimp/commit/649483f97f91a43cae5961a6ca87c1d6cdd021b4
        ** https://github.com/assimp/assimp/commit/f77d0afa550f63ba3c93086669cb57668b7f953c
        **
        ** On écrase donc la transformation du root avec un matrice identité en attendant de voir
        ** s'il faut corriger assip pour la lecture du 3ds ou d'assimp dans sa globalité
        ** Regarder si Celestia n'impose pas des rotations de 90° en rapport avec notre problème
        */
        scene->mRootNode->mTransformation = aiMatrix4x4();

        // Configuration des flags de suppression de composants dans la scène
        // (aiProcess_RemoveComponent)
        _importer->SetPropertyInteger(
            AI_CONFIG_PP_RVC_FLAGS,
            aiComponent_NORMALS
                | aiComponent_TANGENTS_AND_BITANGENTS
                // | aiComponent_COLORS     // kept
                // | aiComponent_TEXCOORDS  // kept
                | aiComponent_BONEWEIGHTS // removed
                | aiComponent_ANIMATIONS  // removed
                // | aiComponent_TEXTURES   // kept
                | aiComponent_LIGHTS  // removed
                | aiComponent_CAMERAS // removed
                                      /// | aiComponent_MESHES    // kept
                                      // | aiComponent_MATERIALS  // kept
        );

        // On applique le post processing après pour s'assurer que la matrice de transformation
        // du root est bien modifiée avant
        //  * aiProcess_Triangulate : obligatoire, voir dans le code car on part du principe qu'on
        //    a des faces de 3 vertices
        //  * aiProcess_GenNormals : obligatoire au minimum pour avoir des normales (smooth ok)
        //  * aiProcess_GenSmoothNormals : Devrait être une option du modèle
        //  * aiProcess_FlipUVs : obligatoire car convention Celestia

        scene = _importer->ApplyPostProcessing(
            aiProcess_Triangulate             // nécessaire
            | aiProcess_JoinIdenticalVertices // conseillé
            | aiProcess_SplitLargeMeshes      // conseillé
            | aiProcess_CalcTangentSpace      // normal mapping
            | aiProcess_GenSmoothNormals      // nécessaire
            | aiProcess_FlipUVs               // nécessaire
            | aiProcess_GenUVCoords           // conseillé
            | aiProcess_TransformUVCoords     // conseillé
            | aiProcess_ValidateDataStructure // Debug (TODO activer le debug)
            | aiProcess_OptimizeMeshes        // conseillé
            | aiProcess_SortByPType           // A tester
            | aiProcess_FixInfacingNormals    // conseillé
            | aiProcess_PreTransformVertices  // supprime les noeuds en appliquant les
                                              // transformations (incompatible
                                              // aiProcess_OptimizeGraph)
            | aiProcess_RemoveComponent // supprime les données du flag AI_CONFIG_PP_RVC_FLAGS
        );
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            std::cerr << "Error processing '" << filename.c_str()
                      << "': " << _importer->GetErrorString() << std::endl;
            delete model;
            return nullptr;
        }

        // Process root node recursively
        processNode(*scene->mRootNode, aiMatrix4x4(), *scene, model, textureLoader);

        // process materials
        for (unsigned int i = 0; i < scene->mNumMaterials; i++)
        {
            auto material = convertAssimpMaterial(*(scene->mMaterials[i]), textureLoader);

            model->addMaterial(std::move(*material));
        }
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << "Error loading model: " << e.what() << std::endl;
        delete model;
    }
    catch (...)
    {
        std::cerr << "Unknown error loading model" << std::endl;
        delete model;
    }

    return model;
}

void
AssimpLoader::processNode(
    const aiNode        &node,
    const aiMatrix4x4   &parentTransform,
    const aiScene       &scene,
    cmod::Model         *model,
    cmod::TextureLoader *textureLoader)
{
    // Current transformation matrix
    aiMatrix4x4 nodeTransform = parentTransform;
    nodeTransform *= node.mTransformation;

    // process all the node's meshes (if any)
    for (unsigned int i = 0; i < node.mNumMeshes; i++)
    {
        aiMesh *mesh = scene.mMeshes[node.mMeshes[i]];

        // process mesh
        if (mesh != nullptr && mesh->mNumFaces > 0)
        {
            auto celmesh = convertAssimpMesh(*mesh, nodeTransform);
            model->addMesh(std::move(*celmesh));
        }
    }

    // then do the same for each of its children
    for (unsigned int i = 0; i < node.mNumChildren; i++)
    {
        processNode(*(node.mChildren[i]), nodeTransform, scene, model, textureLoader);
    }
}

cmod::Material *
AssimpLoader::convertAssimpMaterial(
    const aiMaterial    &assimpMaterial,
    cmod::TextureLoader *textureLoader)
{
    cmod::Material *newMaterial = new cmod::Material();

    aiColor3D diffuse(0.f, 0.f, 0.f);
    assimpMaterial.Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);
    newMaterial->diffuse = cmod::Color(diffuse.r, diffuse.g, diffuse.b);

    float opacity = 1.0f;
    assimpMaterial.Get(AI_MATKEY_OPACITY, opacity);
    newMaterial->opacity = opacity;

    aiColor3D specular(0.f, 0.f, 0.f);
    assimpMaterial.Get(AI_MATKEY_COLOR_SPECULAR, specular);
    newMaterial->specular = cmod::Color(specular.r, specular.g, specular.b);

    float    shininess      = 0.0f;
    aiReturn isShininessSet = assimpMaterial.Get(AI_MATKEY_SHININESS, shininess);

    float    shininessStrength = 0.0f;
    aiReturn isShininessStrengthSet
        = assimpMaterial.Get(AI_MATKEY_SHININESS_STRENGTH, shininessStrength);

    if (isShininessSet == AI_SUCCESS && isShininessStrengthSet == AI_SUCCESS)
    {
        newMaterial->specularPower = shininess * shininessStrength;
    }
    else if (isShininessSet == AI_SUCCESS)
    {
        // See Convert3DSModel
        newMaterial->specularPower
            = static_cast<float>(pow(2.0, 1.0 + 0.1 * static_cast<double>(shininess)));
    }
    if (newMaterial->specularPower > 128.0f)
    {
        newMaterial->specularPower = 128.0f;
    }

    std::vector<std::pair<aiTextureType, cmod::TextureSemantic>> handledTexturesTypes
        = { { aiTextureType_DIFFUSE, cmod::TextureSemantic::DiffuseMap },
            { aiTextureType_NORMALS, cmod::TextureSemantic::NormalMap },
            { aiTextureType_SPECULAR, cmod::TextureSemantic::SpecularMap },
            { aiTextureType_EMISSIVE, cmod::TextureSemantic::EmissiveMap } };

    for (auto textureType : handledTexturesTypes)
    {
        if (assimpMaterial.GetTextureCount(textureType.first) > 0)
        {
            aiString texturePath;
            if (assimpMaterial.GetTexture(
                    textureType.first,
                    0,
                    &texturePath,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr)
                == AI_SUCCESS)
            {
                cmod::TextureResource *tex = nullptr;
                if (textureLoader)
                {
                    tex = textureLoader->loadTexture(texturePath.data);
                }
                else
                {
                    tex = new cmod::DefaultTextureResource(texturePath.data);
                }
                newMaterial->maps[textureType.second] = tex;
            }
        }
    }
    return newMaterial;
}

cmod::Mesh *
AssimpLoader::convertAssimpMesh(const aiMesh &assimpMesh, const aiMatrix4x4 &nodeTransform)
{
    // TODO JEE gérer les modèles vides (0 vertex)
    // gérer les noeuds²

    const unsigned int nVertices    = assimpMesh.mNumVertices;
    const unsigned int nTexCoords   = assimpMesh.mNumVertices; // voir doc mTextureCoords
    bool               hasTexCoords = (assimpMesh.mTextureCoords[0] != nullptr);
    unsigned int       vertexSize
        = 6; // vertices + normals : on force le calcul des normales au chargement
    if (hasTexCoords)
    {
        vertexSize += 2;
    }

    float *vertices = new float[nVertices * vertexSize];

    // Build the vertex list
    for (unsigned int i = 0; i < nVertices; ++i)
    {
        unsigned int k   = i * vertexSize;
        aiVector3D   pos = nodeTransform * assimpMesh.mVertices[i];
        vertices[k + 0]  = pos.x;
        vertices[k + 1]  = pos.y;
        vertices[k + 2]  = pos.z;
        aiVector3D nor   = assimpMesh.mNormals[i];
        vertices[k + 3]  = nor.x;
        vertices[k + 4]  = nor.y;
        vertices[k + 5]  = nor.z;

        if (hasTexCoords)
        {
            aiVector3D texCoord = assimpMesh.mTextureCoords[0][i]; // TODO voir la doc
            vertices[k + 6]     = texCoord.x;
            vertices[k + 7]     = texCoord.y;
        }
    }

    cmod::Mesh::VertexAttribute attributes[8];
    unsigned int                nAttributes = 0;
    unsigned int                offset      = 0;

    // Position attribute is always present
    attributes[nAttributes]
        = cmod::Mesh::VertexAttribute(cmod::Mesh::Position, cmod::Mesh::Float3, 0);
    nAttributes++;
    offset += 12;

    // Normals are computed with the flag aiProcess_GenNormals or aiProcess_GenSmoothNormals
    attributes[nAttributes]
        = cmod::Mesh::VertexAttribute(cmod::Mesh::Normal, cmod::Mesh::Float3, offset);
    nAttributes++;
    offset += 12;

    if (hasTexCoords)
    {
        attributes[nAttributes]
            = cmod::Mesh::VertexAttribute(cmod::Mesh::Texture0, cmod::Mesh::Float2, offset);
        nAttributes++;
        offset += 8;
    }

    // Create the Celestia mesh
    cmod::Mesh *mesh = new cmod::Mesh();
    mesh->setVertexDescription(cmod::Mesh::VertexDescription(offset, nAttributes, attributes));
    mesh->setVertices(nVertices, vertices);

    mesh->setName(assimpMesh.mName.C_Str());

    // An ASSIMP mesh does use only a single material.
    unsigned int nMatGroupFaces = assimpMesh.mNumFaces;

    // Assuming we used aiProcess_Triangulate so we are dealing with triangles
    unsigned int *indices = new unsigned int[nMatGroupFaces * 3];

    for (unsigned int i = 0; i < nMatGroupFaces; i++)
    {
        aiFace face        = assimpMesh.mFaces[i];
        indices[i * 3 + 0] = face.mIndices[0];
        indices[i * 3 + 1] = face.mIndices[1];
        indices[i * 3 + 2] = face.mIndices[2];
    }

    mesh->addGroup(cmod::Mesh::TriList, assimpMesh.mMaterialIndex, nMatGroupFaces * 3, indices);

    return mesh;
}
