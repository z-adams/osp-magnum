/**
 * Open Space Program
 * Copyright © 2019-2020 Open Space Program Project
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#pragma once

#include <string>

#include <MagnumPlugins/TinyGltfImporter/TinyGltfImporter.h>
#include <MagnumPlugins/StbImageImporter/StbImageImporter.h>
#include <Corrade/PluginManager/Manager.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData.h>


#include "Package.h"
#include "PrototypePart.h"

#include "../types.h"
//#include "../scene.h"

namespace osp
{

class AssetImporter
{
typedef Magnum::Trade::TinyGltfImporter TinyGltfImporter;
typedef Corrade::PluginManager::Manager<Magnum::Trade::AbstractImporter>
PluginManager;

public:
    AssetImporter() {}

    static void load_sturdy_file(std::string_view filepath, Package& package);

    /**
     * Load an image from disk at the specified filepath
     * 
     * Loads an ImageData2D into the specified package, but does not create
     * a texture in GPU memory until compile_tex() is called
     * @param filepath [in] string filepath of requested image file
     * @param package [out] Package to put image data into
     */
    static DependRes<Magnum::Trade::ImageData2D> load_image(
        std::string_view filepath, Package& package);

    /**
     * Compile MeshData into an OpenGL Mesh object
     *
     * Takes a MeshData object, compiles it into a Mesh object, and places
     * it into the package
     * 
     * @param meshData [in]  MeshData resource
     * @param package  [out] Package to put Mesh resource into
     * 
     * @return DependRes to the new Mesh resource
     */
    static DependRes<Magnum::GL::Mesh> compile_mesh(
        const DependRes<Magnum::Trade::MeshData> meshData, Package& package);

    /**
     * Fetch and compile MeshData into an OpenGL Mesh object
     *
     * Attempts to retrieve a MeshData with the specified name from the source
     * package, compile it, and place it into the destination package
     * 
     * @param meshDataName [in]  Name of the MeshData resource
     * @param srcPackage   [in]  Package storing the MeshData
     * @param dstPackage   [out] Package to put new Mesh resource into
     * 
     * @return DependRes to the new Mesh resource
     */
    static DependRes<Magnum::GL::Mesh> compile_mesh(
        std::string_view meshDataName, Package& srcPackage, Package& dstPackage);

    /**
     * Compile ImageData2D into an OpenGL Texture2D object
     *
     * Takes the ImageData2D object, compiles it into a Texture2D, and places
     * it into the specified package
     * 
     * @param imageData [in]  ImageData2D resource
     * @param package   [out] Package to put Texture2D resource into
     * 
     * @return DependRes to the new Texture2D resource
     */
    static DependRes<Magnum::GL::Texture2D> compile_tex(
        const DependRes<Magnum::Trade::ImageData2D> imageData, Package& package);

    /**
     * Fetch and compile ImageData2D into an OpenGL Texture2D object
     *
     * Attempts to retrieve an ImageData2D object with the specified name from
     * the source package, compile it, and place it into the destination package
     *
     * @param imageDataName [in]  Name of the ImageData2D resource
     * @param srcPackage    [in]  Package storing the ImageData2D
     * @param dstPackage    [out] Package to put new Texture2D resource into
     *
     * @return DependRes to the new Texture2D resource
     */
    static DependRes<Magnum::GL::Texture2D> compile_tex(
        std::string_view imageDataName, Package& srcPackage, Package& dstPackage);

    enum ECubeMapSideIndex
    {
        PosX = 0,
        NegX = 1,
        PosY = 2,
        NegY = 3,
        PosZ = 4,
        NegZ = 5
    };

    using CubemapImageData_t = std::array<DependRes<Magnum::Trade::ImageData2D>, 6>;
    using CubemapImageNames_t = std::array<std::string_view, 6>;

    /**
     * Compile 6 ImageData2D into an OpenGL CubeMap texture object
     *
     * Takes an ImageData2D for each of positive and negative X, Y, and Z, and
     * compiles it into a CubeMapTexture for use in shaders.
     *
     * OpenGL defines the following ordering for cubemap faces, which is the
     * order in which they should be specified in the input array:
     *
     *  #   Axis   Direction
     * -----------------------
     *  0 |  +X  |  Right
     *  1 |  -X  |  Left
     *  2 |  +Y  |  Top
     *  3 |  -Y  |  Bottom
     *  4 |  +Z  |  Back
     *  5 |  -Z  |  Front
     *
     * @param resName   [in]  The desired name of the resulting CubeMapTexture
     * @param imageData [in]  Array of imagedata to fill cubemap, using OpenGL ordering
     * @param package   [out] Package to put new CubeMapTexture resource into
     *
     * @return DependRes to the new CubeMapTexture resource
     */
    static DependRes<Magnum::GL::CubeMapTexture> compile_cubemap(
        std::string_view resName,
        CubemapImageData_t const& imageData,
        Package& package);

    /**
     * Fetch and compile ImageData2D into an OpenGL CubeMapTexture
     * 
     * Takes an array of names corresponding to ImageData2D objects in the
     * source package, compiles them into a cubemap texture, and stores it in
     * the destination package.
     * 
     * OpenGL defines the following ordering for cubemap faces, which is the
     * order in which they should be specified in the input array:
     *
     *  #   Axis   Direction
     * -----------------------
     *  0 |  +X  |  Right
     *  1 |  -X  |  Left
     *  2 |  +Y  |  Top
     *  3 |  -Y  |  Bottom
     *  4 |  +Z  |  Back
     *  5 |  -Z  |  Front
     * 
     * @param resName        [in]  The desired name of the resulting CubeMapTexture
     * @param imageDataNames [in]  Array of names of ImageData2D objects to fill
                                   cubemap, using OpenGL ordering
     * @param srcPackage     [out] Package to fetch image data from
     * @param dstPackage     [out] Package to put new CubeMapTexture resource into
     * 
     * @return DependRes to the new CubeMapTexture resource
     */
    static DependRes<Magnum::GL::CubeMapTexture> compile_cubemap(
        std::string_view resName,
        CubemapImageNames_t imgDataNames,
        Package& srcPackage, Package& dstPackage);
private:

    /**
     * Load machines from node extras
     * 
     * Each node in the glTF tree may possess machines, but only the root
     * PrototypePart stores them. The PrototypePart's machineArray is passed,
     * alongside the current node (object)'s machineIndexArray.
     * PrototypeMachines are added to the PrototypePart's master list, and the
     * index of each machine is added to the machineIndexArray so that the
     * current node/object can keep track of which machines belong to it.
     *
     * @param extras            [in] An extras node from a glTF file
     * @param machineArray      [out] A machine array from a PrototypePart
     * 
     * @return A machineIndexArray which is used by PrototypeObjects to store
     * the indices of the machineArray elements which belong to it
     */
    static std::vector<unsigned> load_machines(tinygltf::Value const& extras,
        std::vector<PrototypeMachine>& machineArray);

    /**
     * Load only associated config files, and add resource paths to the package
     * But for now, this function just loads everything.
     *
     * @param gltfImporter [in] glTF importer referencing opened sturdy file
     * @param resPrefix [in] Unique name associated with the data source,
     *       used to make resource names (e.g. mesh) unique to avoid collisions
     * @param package [out] Package to put resource paths into
     */
    static void load_sturdy(TinyGltfImporter& gltfImporter,
            std::string_view resPrefix, Package& package);

    /**
     * Load a part from a sturdy
     *
     * Reads the config from the node with the specified ID into a PrototypePart
     * and stores it in the specified package
     *
     * @param gltfImpoter [in] importer used to read node data
     * @param package [out] package which receives loaded data
     * @param id [in] ID of node containing part information
     * @param resPrefix [in] Unique prefix for mesh names (see load_sturdy())
     */
    static void load_part(TinyGltfImporter& gltfImporter,
        Package& package, Magnum::UnsignedInt id, std::string_view resPrefix);

    /**
     * Load a plume object from a sturdy
     *
     * Reads the config from the node with the specified ID into a PlumeEffectData
     * and stores it in the specified package
     *
     * @param gltfImpoter [in] importer used to read node data
     * @param package [out] package which receives loaded data
     * @param id [in] ID of node containing plume information
     */
    static void load_plume(TinyGltfImporter& gltfImporter,
        Package& package, Magnum::UnsignedInt id, std::string_view resPrefix);

    static void proto_add_obj_recurse(TinyGltfImporter& gltfImporter,
                               Package& package,
                               std::string_view resPrefix,
                               PrototypePart& part,
                               Magnum::UnsignedInt parentProtoIndex,
                               Magnum::UnsignedInt childGltfIndex);

};

}
