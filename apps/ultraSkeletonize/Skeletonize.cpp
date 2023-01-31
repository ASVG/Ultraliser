/***************************************************************************************************
 * Copyright (c) 2016 - 2022
 * Blue Brain Project (BBP) / Ecole Polytechnique Federale de Lausanne (EPFL)
 *
 * Author(s)
 *      Marwan Abdellah < marwan.abdellah@epfl.ch >
 *
 * This file is part of Ultraliser < https://github.com/BlueBrain/Ultraliser >
 *
 * This library is free software; you can redistribute it and/or modify it under the terms of the
 * GNU General Public License version 3.0 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License along with this library;
 * if not, write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
 * MA 02111-1307, USA.
 * You can also find it on the GNU web site < https://www.gnu.org/licenses/gpl-3.0.en.html >
 **************************************************************************************************/

#include <Ultraliser.h>
#include <AppCommon.h>
#include <AppArguments.h>
#include <data/meshes/simple/IcoSphere.h>


namespace Ultraliser
{

AppOptions* parseArguments(const int& argc , const char** argv)
{
    std::unique_ptr< AppArguments > args = std::make_unique <AppArguments>(argc, argv,
              "This tool reconstructs a watertight polygonal mesh from an input "
              "non-watertight mesh. The generated mesh can be also optimized to "
              "reduce the number of triangles while preserving the volume. "
              "The output mesh is guaranteed in all cases to be two-manifold");

    args->addInputMeshArguments();
    args->addOutputArguments();
    args->addVolumeArguments();
    args->addMeshArguments();
    args->addSuppressionArguments();
    args->addDataArguments();
    args->addProcessingArguments();

    // Get all the options
    AppOptions* options = args->getOptions();

    LOG_TITLE("Creating Context");

    // Verify the arguments after parsing them and extracting the application options.
    options->verifyInputMeshArgument();
    options->verifyOutputDirectoryArgument();
    options->verifyBoudsFileArgument();
    options->verifyMeshExportArguments();
    options->verifyMeshPrefixArgument();
    options->verifyIsoSurfaceExtractionArgument();
    options->verifyProcessingArguments();

    // Initialize context, once everything is in place and all the options are verified
    options->initializeContext();

    // Return the executable options
    return options;
}


void run(int argc , const char** argv)
{
    // Parse the arguments and get the tool options
    auto options = parseArguments(argc, argv);

    // Load the input mesh
    auto inputMesh = loadInputMesh(options);



   //





    auto prefix = options->projectionPrefix;

    // Create the volume from the mesh
    auto solidVolume = createVolumeGrid(inputMesh, options);

    // Surface voxelization
    solidVolume->surfaceVoxelization(inputMesh, true, true);
    solidVolume->solidVoxelization(options->voxelizationAxis);
    solidVolume->project(prefix + "_solid",
                    options->projectXY, options->projectXZ, options->projectZY,
                    options->projectColorCoded);

<<<<<<< HEAD

    Vector3f somaCenter;
    float somaRadius;

    solidVolume->applyThinning(somaCenter, somaRadius);


    // Map
    Mesh* sample = new IcoSphere(3);
    sample->scale(somaRadius, somaRadius, somaRadius);
    sample->translate(somaCenter);

    sample->map(inputMesh);

    sample->exportMesh(options->volumePrefix, true);

    solidVolume->surfaceVoxelization(sample, true, true);


=======
    Vector3f pMinV, pMaxV;
    solidVolume->applyThinning(pMinV, pMaxV);

    //solidVolume->clear();

    //pMinV.print();
    //pMaxV.print();


    //solidVolume->surfaceVoxelizationReion(inputMesh, pMinV, pMaxV, false);
    //solidVolume->solidVoxelization(options->voxelizationAxis);
>>>>>>> Minor.



    solidVolume->project(prefix + "_thin",
                    options->projectXY, options->projectXZ, options->projectZY,
                    options->projectColorCoded);

    solidVolume->writeVolumes(options->volumePrefix,
                         options->exportBitVolume,
                         options->exportUnsignedVolume,
                         options->exportFloatVolume,
                         options->exportNRRDVolume,
                         options->exportRawVolume);
}
}

int main(int argc , const char** argv)
{
    TIMER_SET;

    Ultraliser::run(argc, argv);

    LOG_STATUS_IMPORTANT("Ultralization Stats.");
    LOG_STATS(GET_TIME_SECONDS);

    ULTRALISER_DONE;
}
