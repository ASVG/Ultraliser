#ifndef OPTIONS_HH
#define OPTIONS_HH

#include <common/Common.h>
#include <data/volumes/Volumes.h>

namespace Ultraliser
{

/**
 * @brief The Options struct
 */
struct Options
{
    /**
     * @brief inputMorphology
     * Input mesh file
     */
    std::string inputMorphology;

    /**
     * @brief outputDirectory
     * The directory where the volume will be created.
     * Output directory
     */
    std::string outputDirectory;

    /**
     * @brief boundsFile
     * Use a bounds file to only voxelize part of the mesh or even a greater
     * space. If the file is not given, the bounding box of the input mesh will
     * be used in addition to a little delta to avoid intersection.
     */
    std::string boundsFile;

    /**
     * @brief volumeResolution
     * The base resolution of the volume that corresponds to the largest
     * dimension.
     */
    uint64_t volumeResolution;

    /**
     * @brief autoResolution
     * Sets the resolution of the volume based on mesh dimensions.
     */
    bool autoResolution;

    /**
     * @brief edgeGap
     */
    float edgeGap;

    /**
     * @brief solid
     * Use solid voxelization to fill the volume.
     */
    bool useSolidVoxelization;

    /**
     * @brief solid
     * Fill the interior of the volume using solid voxelization.
     */
    Volume::SOLID_VOXELIZATION_AXIS VoxelizationAxis;

    /**
     * @brief volumeType
     * Use a specific volume for the voxelization process.
     */
    std::string volumeType;

    /**
     * @brief projectXY
     * If this flag is set, the XY projection (Z-axis) of the volume will be saved to an image.
     * This flag is set to validate the output volume.
     */
    bool projectXY;

    /**
     * @brief projectXZ
     * If this flag is set, the XZ projection (Y-axis) of the volume will be saved to an image.
     * This flag is set to validate the output volume.
     */
    bool projectXZ;

    /**
     * @brief projectZY
     * If this flag is set, the ZY projection (X-axis) of the volume will be saved to an image.
     * This flag is set to validate the output volume.
     */
    bool projectZY;

    /**
     * @brief projectColorCoded
     * If this flag is set, a series of color-coded projections with different color maps will
     * be generated.
     */
    bool projectColorCoded;

    /**
     * @brief stackXY
     * Create an image stack along the XY plane.
     */
    bool stackXY;

    /**
     * @brief stackXZ
     * Create an image stack along the XZ plane.
     */
    bool stackXZ;

    /**
     * @brief stackZY
     * Create an image stack along the ZY plane.
     */
    bool stackZY;

    /**
     * @brief createBinaryVolume
     * If this flag is set, a binary volume will be created. This volume has
     * 1 bit per voxel.
     */
    bool writeBitVolume;

    /**
     * @brief createByteVolume
     * If this flag is set, a default raw volume will be created.This volume
     * has 1 byte per voxel.
     */
    bool writeByteVolume;

    /**
     * @brief writeNRRDVolume
     * If this flag is set, the volume will be written to an NRRD file that is
     * compatible with VTK.
     */
    bool writeNRRDVolume;

    /**
     * @brief optimizeMesh
     * Optimize the reconstructed mesh. The reconstruct mesh flag must be set
     * for this option to work.
     */
    bool optimizeMesh;

    /**
     * @brief smoothingIterations
     * Number of iterations required to smooth the optimized mesh, by default 10.
     */
    int64_t smoothingIterations;

    /**
     * @brief smoothingFactor
     * The rate at which the unnecessary vertices will be removed from the
     * optimized mesh. This fator has impact on the mesh size. The higher this
     * factor is the lower the mesh size becomes. By default 10.
     */
    float smoothingFactor;

    /**
     * @brief exportOBJ
     * Export any reconstructed mesh to .OBJ file.
     */
    bool exportOBJ;

    /**
     * @brief exportPLY
     * Export any reconstructed mesh to .PLY file.
     */
    bool exportPLY;

    /**
     * @brief exportOFF
     * Export any reconstructed mesh to .OFF file.
     */
    bool exportOFF;

    /**
     * @brief exportSTL
     * Export any reconstructed mesh to .STL file.
     */
    bool exportSTL;

    /**
     * @brief exportVolumeMesh
     * Export a mesh that represents the volume where each voxel will be represented by a cube.
     */
    bool exportVolumeMesh;

    /**
     * @brief prefix
     * Just a prefix that will be used to label the output files. If this
     * is not given by the user, the name of the mesh file will be used.
     */
    std::string prefix;

    /**
     * @brief writeStatictics
     * Write the statictics.
     */
    bool writeStatistics;

    /**
     * @brief writeDistributions
     * Write distributions of morphologies and meshes.
     */
    bool writeDistributions;

    /**
     * @brief outputPrefix
     * Simply, the [OUTPUT_DIRECTORY]/[PREFIX]. This variable is just added to
     * make the code simpler.
     */
    std::string outputPrefix;

    /**
     * @brief ignoreDMCMesh
     * If this flag is set, the resulting mesh from the DMC stage will be
     * ignored and not exported to disk. Note that if this flag is set and
     * the ignoreSelfIntersection flag is set as well, there will no be
     * any exported mesh from this process.
     */
    bool ignoreDMCMesh;

    /**
     * @brief ignoreSelfIntersections
     * Ignore if the mesh has self intersections, and do NOT repair them.
     */
    bool ignoreSelfIntersections;

    /**
     * @brief ignoreOptimizedNonWatertightMesh
     * Ignores writing the optimized mesh that is not watertight.
     */
    bool ignoreOptimizedNonWatertightMesh;
};
};

}

#endif // OPTIONS_HH
