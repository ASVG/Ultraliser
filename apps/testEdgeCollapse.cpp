#include <Ultraliser.h>

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: ./testEdgeCollapse <path to mesh> <path to folder> <collapse iteration count>" << std::endl;
        return 0;
    }

    auto path = std::string(argv[1]);
    auto result = std::string(argv[2]);
    auto percentage = std::string(argv[3]);
    auto percentageNumber = std::stof(percentage);

    std::cout << "Applying " << percentage << "% edge collapse removal" << std::endl;

    auto inputMesh = Ultraliser::Mesh(path);
    inputMesh.collapseEdges(percentageNumber);

    auto savePath = result + "/collapsed_" + percentage;
    inputMesh.exportMesh(savePath, true);

    inputMesh.optimizeUsingDefaultParameters();

    inputMesh.exportMesh(savePath + "_optimized", true);
    return 0;
}