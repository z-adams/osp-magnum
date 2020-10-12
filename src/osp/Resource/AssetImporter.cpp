#include <iostream>

#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/TextureData.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/ImageView.h>
#include <Magnum/Trade/MaterialData.h>
#include <Magnum/Trade/PbrMetallicRoughnessMaterialData.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/Mesh.h>


#include <MagnumExternal/TinyGltf/tiny_gltf.h>

#include "Package.h"
#include "AssetImporter.h"
#include "adera/Plume.h"

using Corrade::Containers::Optional;
using Magnum::Trade::ImageData2D;
using Magnum::Trade::MeshData;
using Magnum::Trade::SceneData;
using Magnum::GL::Texture2D;
using Magnum::GL::Mesh;

namespace osp
{

void osp::AssetImporter::load_sturdy_file(std::string const& filepath, Package& pkg)
{
    PluginManager pluginManager;
    TinyGltfImporter gltfImporter{pluginManager};

    // Open .sturdy.gltf file
    gltfImporter.openFile(filepath);
    if (!gltfImporter.isOpened() || gltfImporter.defaultScene() == -1)
    {
        std::cout << "Error: couldn't open file: " << filepath << "\n";
    }

    std::string dataResourceName = filepath.substr(0, filepath.find('.')) + ':';

    load_sturdy(gltfImporter, dataResourceName, pkg);

    gltfImporter.close();
}

void AssetImporter::load_machines(tinygltf::Value const& extras,
    std::vector<PrototypeMachine>& machineArray,
    std::vector<unsigned>& machineIndexArray)
{
    if (!extras.Has("machines"))
    {
        std::cout << "Error: no machines found!\n";
        return;
    }
    tinygltf::Value const& machines = extras.Get("machines");
    std::cout << "JSON machines!\n";
    auto const& machArray = machines.Get<tinygltf::Value::Array>();

    // Loop through machine configs
            // machArray looks like:
            // [
            //    { "type": "Rocket", stuff... },
            //    { "type": "Control", stuff...}
            // ]
    for (tinygltf::Value const& value : machArray)
    {
        std::string type = value.Get("type").Get<std::string>();
        std::cout << "test: " << type << "\n";

        if (type.empty())
        {
            continue;
        }

        PrototypeMachine machine;
        machine.m_type = std::move(type);

        for (std::string key : value.Keys())
        {
            std::cout << "Read config '" << key << "' : ";
            tinygltf::Value v = value.Get(key);
            switch (v.Type())
            {
            case tinygltf::Type::REAL_TYPE:
            {
                double val = v.Get<double>();
                machine.m_config.emplace(key, val);
                std::cout << val << " (real)\n";
                break;
            }

            case tinygltf::Type::INT_TYPE:
            {
                int val = v.Get<int>();
                machine.m_config.emplace(key, val);
                std::cout << val << " (int)\n";
                break;
            }
            case tinygltf::Type::STRING_TYPE:
            {
                std::string val = v.Get<std::string>();
                machine.m_config.emplace(key, val);
                std::cout << val << "(string)\n";
                break;
            }
            default:
                std::cout << "(UNKNOWN)\n";
                break;
            }
        }

        machineIndexArray.push_back(machineArray.size());
        machineArray.emplace_back(std::move(machine));
    }
}

void osp::AssetImporter::load_part(TinyGltfImporter& gltfImporter,
    Package& pkg, unsigned id, std::string const& resPrefix)
{
    // It's a part
    std::cout << "PART!\n";

    // Recursively add child nodes to part
    PrototypePart part;
    proto_add_obj_recurse(gltfImporter, pkg, resPrefix, part, 0, id);

    // Parse extra properties
    tinygltf::Node const& node = *static_cast<tinygltf::Node const*>(
        gltfImporter.object3D(id)->importerState());

    tinygltf::Value const& extras = node.extras;

    part.get_mass() = extras.Get("massdry").Get<double>();

    pkg.add<PrototypePart>(gltfImporter.object3DName(id), std::move(part));
}

void osp::AssetImporter::load_plume(TinyGltfImporter& gltfImporter,
    Package& pkg, unsigned id, std::string const& resPrefix)
{
    using Corrade::Containers::Pointer;
    using Magnum::Trade::ObjectData3D;
    using namespace tinygltf;

    std::cout << "Plume! Node \"" << gltfImporter.object3DName(id) << "\"\n";

    // Get mesh data
    Pointer<ObjectData3D> rootNode = gltfImporter.object3D(id);
    unsigned meshID = gltfImporter.object3D(rootNode->children()[0])->instance();
    std::string meshName = resPrefix + gltfImporter.meshName(meshID);

    // Get shader params from extras
    Node const& node = *static_cast<Node const*>(
        gltfImporter.object3D(id)->importerState());

    Value const& extras = node.extras;
    float flowVel = extras.Get("flowvelocity").Get<double>();
    Magnum::Color4 color;
    Value::Array colorArray = extras.Get("color").Get<Value::Array>();
    for (int i = 0; i < 4; i++)
    {
        Value const& component = colorArray[i];
        color[i] = component.GetNumberAsDouble();
    }
    float zMax = extras.Get("zMax").Get<double>();
    float zMin = extras.Get("zMin").Get<double>();

    PlumeEffectData plumeData;
    plumeData.meshName = meshName;
    plumeData.flowVelocity = flowVel;
    plumeData.color = color;
    plumeData.zMax = zMax;
    plumeData.zMin = zMin;

    pkg.add<PlumeEffectData>(gltfImporter.object3DName(id), std::move(plumeData));
}


/* Explanation of resPrefix:

   When a mesh is created in blender, the object itself has a name (the one that
   shows up in the scene hierarchy), but the underlying mesh data within that
   object actually has a unique name, usually the name of the primitive that was
   used initially, unless it was changed. These names will be something like
   "Cylinder.004" and are numbered to prevent name collisions within a blend 
   file. The issue is that multiple blend files can have a "Cylinder.004" and
   unless the author renames the mesh object itself (which most people never
   touch), when you export the scenes to glTF files and load them both here, you
   end up with a resource key collision due to the repeated name. Passing
   resPrefix around allows us to specify a unique prefix to prepend to the mesh
   name (or any other resource that has the same problem) that is used
   internally to avoid name conflicts.

*/
void AssetImporter::load_sturdy(TinyGltfImporter& gltfImporter,
    std::string const& resPrefix, Package& pkg)
{
    std::cout << "Found " << gltfImporter.object3DCount() << " nodes\n";
    Optional<SceneData> sceneData = gltfImporter.scene(gltfImporter.defaultScene());
    if (!sceneData)
    {
        std::cout << "Error: couldn't load scene data\n";
        return;
    }

    // Loop over and discriminate all top-level nodes
    // Currently, part_* are the only nodes that necessitate special handling
    for (unsigned childID : sceneData->children3D())
    {
        const std::string& nodeName = gltfImporter.object3DName(childID);
        std::cout << "Found node: " << nodeName << "\n";

        if (nodeName.compare(0, 5, "part_") == 0)
        {
            load_part(gltfImporter, pkg, childID, resPrefix);
        }
        else if (nodeName.compare(0, 6, "plume_") == 0)
        {
            load_plume(gltfImporter, pkg, childID, resPrefix);
        }
    }

    // Load all associated mesh data
    // Temporary: eventually if would be preferable to retrieve the mesh names only
    for (unsigned i = 0; i < gltfImporter.meshCount(); i++)
    {
        using Magnum::MeshPrimitive;

        std::string const& meshName = resPrefix + gltfImporter.meshName(i);
        std::cout << "Mesh: " << meshName << "\n";

        Optional<MeshData> meshData = gltfImporter.mesh(i);
        if (!meshData || (meshData->primitive() != MeshPrimitive::Triangles))
        {
            std::cout << "Error: mesh " << meshName
                << " not composed of triangles\n";
            continue;
        }
        pkg.add<MeshData>(meshName, std::move(*meshData));
    }

    // Load all associated image data
    // Temporary: eventually it would be preferable to retrieve the URIs only
    for (unsigned i = 0; i < gltfImporter.textureCount(); i++)
    {
        auto imgID = gltfImporter.texture(i)->image();
        std::string const& imgName = gltfImporter.image2DName(imgID);
        std::cout << "Loading image: " << imgName << "\n";

        Optional<ImageData2D> imgData = gltfImporter.image2D(imgID);
        if (!imgData)
        {
            continue;
        }
        pkg.add<ImageData2D>(imgName, std::move(*imgData));
    }
}

DependRes<ImageData2D> osp::AssetImporter::load_image(
    const std::string& filepath, Package& pkg)
{
    using Magnum::Trade::AbstractImporter;
    using Corrade::PluginManager::Manager;
    using Corrade::Containers::Pointer;

    Manager<AbstractImporter> manager;
    Pointer<AbstractImporter> importer
        = manager.loadAndInstantiate("AnyImageImporter");
    if (!importer || !importer->openFile(filepath))
    {
        std::cout << "Error: could not open file " << filepath << "\n";
        return DependRes<ImageData2D>();
    }

    Optional<ImageData2D> image = importer->image2D(0);
    if (!image)
    {
        std::cout << "Error: could not read image in file " << filepath << "\n";
        return DependRes<ImageData2D>();
    }

    return pkg.add<ImageData2D>(filepath, std::move(*image));
}

DependRes<Mesh> osp::AssetImporter::compile_mesh(
    const DependRes<MeshData> meshData, Package& pkg)
{
    using Magnum::GL::Mesh;

    if (meshData.empty())
    {
        std::cout << "Error: requested MeshData resource \"" << meshData.name()
            << "\" not found\n";
        return DependRes<Mesh>();
    }

    return pkg.add<Mesh>(meshData.name(), Magnum::MeshTools::compile(*meshData));
}

DependRes<Magnum::GL::Mesh> AssetImporter::compile_mesh(
    const std::string& meshDataName, Package& package)
{
    DependRes<MeshData> meshData = package.get<MeshData>(meshDataName);
    if (meshData.empty())
    {
        return DependRes<Mesh>();
    }
    return compile_mesh(meshData, package);
}

DependRes<Texture2D> osp::AssetImporter::compile_tex(
    const DependRes<ImageData2D> imageData, Package& package)
{
    using Magnum::GL::SamplerWrapping;
    using Magnum::GL::SamplerFilter;
    using Magnum::GL::textureFormat;

    if (imageData.empty())
    {
        std::cout << "Error: requested ImageData2D resource \"" << imageData.name()
            << "\" not found\n";
        return DependRes<Texture2D>();
    }

    Magnum::ImageView2D view = *imageData;

    Magnum::GL::Texture2D tex;
    tex.setWrapping(SamplerWrapping::ClampToEdge)
        .setMagnificationFilter(SamplerFilter::Linear)
        .setMinificationFilter(SamplerFilter::Linear)
        .setStorage(1, textureFormat((*imageData).format()), (*imageData).size())
        .setSubImage(0, {}, view);

    return package.add<Texture2D>(imageData.name(), std::move(tex));
}

DependRes<Magnum::GL::Texture2D> AssetImporter::compile_tex(
    const std::string& imageDataName, Package& package)
{
    DependRes<ImageData2D> imgData = package.get<ImageData2D>(imageDataName);
    if (imgData.empty())
    {
        return DependRes<Magnum::GL::Texture2D>();
    }
    return compile_tex(imgData, package);
}

//either an appendable package, or
void AssetImporter::proto_add_obj_recurse(TinyGltfImporter& gltfImporter, 
                                           Package& package,
                                           std::string const& resPrefix,
                                           PrototypePart& part,
                                           unsigned parentProtoIndex,
                                           unsigned childGltfIndex)
{
    using Corrade::Containers::Pointer;
    using Corrade::Containers::Optional;
    using Magnum::Trade::ObjectData3D;
    using Magnum::Trade::MeshObjectData3D;
    using Magnum::Trade::ObjectInstanceType3D;
    using Magnum::Trade::MaterialData;
    using Magnum::Trade::MaterialType;
    using Magnum::Trade::PbrMetallicRoughnessMaterialData;

    // Add the object to the prototype
    Pointer<ObjectData3D> childData = gltfImporter.object3D(childGltfIndex);
    std::vector<PrototypeObject>& protoObjects = part.get_objects();
    const std::string& name = gltfImporter.object3DName(childGltfIndex);

    // I think I've been doing too much C
    PrototypeObject obj;
    obj.m_parentIndex = parentProtoIndex;
    obj.m_childCount = childData->children().size();
    obj.m_translation = childData->translation();
    obj.m_rotation = childData->rotation();
    obj.m_scale = childData->scaling();
    obj.m_type = ObjectType::NONE;
    obj.m_name = name;

    std::cout << "Adding obj to Part: " << name << "\n";
    int meshID = childData->instance();

    bool hasMesh = (
            childData->instanceType() == ObjectInstanceType3D::Mesh
            && meshID != -1);

    if (name.compare(0, 4, "col_") == 0)
    {
        // It's a collider
        obj.m_type = ObjectType::COLLIDER;

        // do some stuff here
        tinygltf::Node const& node = *static_cast<tinygltf::Node const*>(
        childData->importerState());
        tinygltf::Value const& extras = node.extras;

        std::string const& shapeName = extras.Get("shape").Get<std::string>();
        ECollisionShape shape;
        if (shapeName == "cylinder")
        {
            shape = ECollisionShape::CYLINDER;
        }
        else
        {
            shape = ECollisionShape::BOX;
        }
        obj.m_objectData = ColliderData{shape};

        std::cout << "obj: " << name << " is a \"" << shapeName << "\" collider\n";
    }
    else if (hasMesh)
    {
        // It's a drawable mesh
        const std::string& meshName = resPrefix + gltfImporter.meshName(meshID);
        std::cout << "obj: " << name << " uses mesh: " << meshName << "\n";
        obj.m_type = ObjectType::MESH;

        // The way it's currently set up is that the mesh's names are the same
        // as their resource paths. So the resource path is added to the part's
        // list of strings, and the object's mesh is set to the index to that
        // string.
        obj.m_objectData = DrawableData{
            static_cast<unsigned>(part.get_strings().size())};
        part.get_strings().push_back(meshName);

        MeshObjectData3D& mesh = static_cast<MeshObjectData3D&>(*childData);
        Pointer<MaterialData> mat = gltfImporter.material(mesh.material());

        if (mat->types() & MaterialType::PbrMetallicRoughness)
        {
            const auto& pbr = mat->as<PbrMetallicRoughnessMaterialData>();

            auto imgID = gltfImporter.texture(pbr.baseColorTexture())->image();
            std::string const& imgName = gltfImporter.image2DName(imgID);
            std::cout << "Base Tex: " << imgName << "\n";
            std::get<DrawableData>(obj.m_objectData).m_textures.push_back(
                static_cast<unsigned>(part.get_strings().size()));
            part.get_strings().push_back(imgName);

            if (pbr.hasNoneRoughnessMetallicTexture())
            {
                imgID = gltfImporter.texture(pbr.metalnessTexture())->image();
                std::cout << "Metal/rough texture: "
                    << gltfImporter.image2DName(imgID) << "\n";
            } else
            {
                std::cout << "No metal/rough texture found for " << name << "\n";
            }
            
        } else
        {
            std::cout << "Error: unsupported material type\n";
        }
    }

    // Check for and read machines
    tinygltf::Node const& node =
        *static_cast<tinygltf::Node const*>(childData->importerState());
    if (node.extras.Has("machines"))
    {
        load_machines(node.extras, part.get_machines(), obj.m_machineIndices);
    }

    int objIndex = protoObjects.size();
    protoObjects.push_back(std::move(obj));

    for (unsigned childId: childData->children())
    {
        proto_add_obj_recurse(gltfImporter, package, resPrefix, part, objIndex, childId);
    }
}

}

