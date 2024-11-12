#include <gz/msgs/heightmap.pb.h>
#include <gz/rendering.hh>
#include <gz/common/Util.hh>
#include <gz/common/Filesystem.hh>
#include <gz/transport/Node.hh>
#include <memory>

int main() {
    // Load the Gazebo rendering library
    gz::rendering::RenderingIface* rendering = gz::rendering::Factory::NewRenderingIface();
    if (!rendering) {
        std::cerr << "Error: Could not create rendering interface." << std::endl;
        return 1;
    }

    // Load the heightmap image
    auto data = std::make_shared<gz::common::ImageHeightmap>();
    data->Load(gz::common::joinPaths("/images", "test.png"));

    // Define the heightmap descriptor
    gz::rendering::HeightmapDescriptor desc;
    desc.SetName("example_bowl");
    desc.SetData(data);
    desc.SetSize({20, 20, 5});  // Set the size of the terrain
    desc.SetSampling(2u);  // Adjust sampling for better detail
    desc.SetUseTerrainPaging(false);

    // // Create textures for the heightmap
    // gz::rendering::HeightmapTexture textureA;
    // textureA.SetSize(1.0);
    // textureA.SetDiffuse("path_to_your_texture/dirt_diffusespecular.png");
    // textureA.SetNormal("path_to_your_texture/flat_normal.png");
    // desc.AddTexture(textureA);

    // Add blending effect for the terrain
    gz::rendering::HeightmapBlend blendA;
    blendA.SetMinHeight(2.0);
    blendA.SetFadeDistance(5.0);
    desc.AddBlend(blendA);

    // Create the heightmap geometry in the scene
    auto heightmapGeom = rendering->CreateHeightmap(desc);

    // Create a visual for the heightmap
    auto vis = rendering->CreateVisual();
    vis->AddGeometry(heightmapGeom);

    // Add visual to the root of the scene
    auto root = rendering->Root();
    root->AddChild(vis);

    std::cout << "Heightmap created and added to scene." << std::endl;
    return 0;
}
