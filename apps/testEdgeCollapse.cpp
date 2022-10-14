#include <Ultraliser.h>

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout << "Usage: ./testEdgeCollapse <path to mesh> <collapse iteration count>" << std::endl;
        return 0;
    }

    auto path = std::string(argv[1]);
    auto iterations = std::string(argv[2]);
    auto iterationCount = std::stoul(iterations);

    std::cout << "Applying " << iterationCount << " edge collapse iterations" << std::endl;
    
    auto inputMesh = Ultraliser::Mesh(path);
    inputMesh.collapseEdges(iterationCount);
    
    auto savePath = std::string("/home/nadir/Desktop/collapsed_") + iterations;
    inputMesh.exportMesh(savePath, true);

    inputMesh.optimizeUsingDefaultParameters();

    inputMesh.exportMesh(savePath + "_optimized", true);
    return 0;
}