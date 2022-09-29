/***************************************************************************************************
 * Copyright (c) 2016 - 2021
 * Blue Brain Project (BBP) / Ecole Polytechnique Federale de Lausanne (EPFL)
 *
 * Author(s)
 *      Marwan Abdellah < marwan.abdellah@epfl.ch >
 *      Juan Jose Garcia Cantero < juanjose.garcia@epfl.ch>
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

#include "NeuronMorphology.h"
#include <data/morphologies/swc/NeuronSWCReader.h>
#include <data/morphologies/h5/NeuronH5Reader.h>
#include <utilities/String.h>
#include <stack>

namespace Ultraliser
{

NeuronMorphology::NeuronMorphology(const NeuronSWCSamples& swcSamples)
{
    // Construct the morphology from the loaded data
    _constructMorphologyFromSWC(swcSamples);
}

NeuronMorphology::NeuronMorphology(const H5Samples& h5Samples, const H5Sections& h5Sections)
{
    // Construct the morphology from the loaded data
    _constructMorphologyFromH5(h5Samples, h5Sections);
}

void NeuronMorphology::trim(size_t axonBranchOrder,
                            size_t basalBranchOrder,
                            size_t apicalBranchOrder)
{
    Sections newSections;
    Sections newFirstSections;
    _samples.clear();
    _pMin = Vector3f(std::numeric_limits<float>::max());
    _pMax = Vector3f(std::numeric_limits<float>::lowest());
 
    for (uint8_t i = 0; i < _firstSections.size(); ++i)
    {
        size_t maxDepth = 0;
        switch (_firstSections[i]->getType())
        {
        case PROCESS_TYPE::NEURON_AXON:
            maxDepth = axonBranchOrder;
            break;
        case PROCESS_TYPE::NEURON_BASAL_DENDRITE:
            maxDepth = basalBranchOrder;
            break;
        case PROCESS_TYPE::NEURON_APICAL_DENDRITE:
            maxDepth = apicalBranchOrder;
            break;
        case PROCESS_TYPE::SOMA:
        case PROCESS_TYPE::GLIA_PERIVASCULAR_PROCESS:
        case PROCESS_TYPE::GLIA_PROCESS:
        case PROCESS_TYPE::VASCULATURE:
        case PROCESS_TYPE::UNKNOWN_PROCESS:
        default:
            break;
        }
        
        std::stack< std::pair< Section*, size_t > > sectionStack;
        if (maxDepth > 0)
        {        
            sectionStack.push(std::make_pair(_firstSections[i], 0));
            newFirstSections.push_back(_firstSections[i]);
        }

        while(!sectionStack.empty())
        {
            auto section = sectionStack.top().first;
            auto depth = sectionStack.top().second;
            sectionStack.pop();
            
            auto childrenIndices = section->getChildrenIndices(); 
            auto parentsIndices = section->getParentIndices();

            for (auto index: childrenIndices)
                sectionStack.push(std::make_pair(_sections[index], depth + 1));

            if (depth < maxDepth)
            {
                size_t index = newSections.size();
                section->setIndex(index);
                section->clearChildrenIndices();
                newSections.push_back(section);
                for (auto childIndex: childrenIndices)
                {
                    auto childSection = _sections[childIndex];
                    childSection->clearParentsIndices();
                    childSection->addParentIndex(index);
                }   
                for (auto parentIndex: parentsIndices)
                {
                    auto parentSection = newSections[parentIndex];
                    parentSection->addChildIndex(index);
                }
               
                auto sectionSamples = section->getSamples();
                _samples.insert(_samples.end(), sectionSamples.begin(), sectionSamples.end());
                for (auto sample: sectionSamples)
                {
                    Vector3f pMaxSample = sample->getPosition() + Vector3f(sample->getRadius());
                    Vector3f pMinSample = sample->getPosition() - Vector3f(sample->getRadius());

                    if (pMaxSample.x() > _pMax.x()) _pMax.x() = pMaxSample.x();
                    if (pMaxSample.y() > _pMax.y()) _pMax.y() = pMaxSample.y();
                    if (pMaxSample.z() > _pMax.z()) _pMax.z() = pMaxSample.z();

                    if (pMinSample.x() < _pMin.x()) _pMin.x() = pMinSample.x();
                    if (pMinSample.y() < _pMin.y()) _pMin.y() = pMinSample.y();
                    if (pMinSample.z() < _pMin.z()) _pMin.z() = pMinSample.z();
                }
            }
            else
                delete section;
        }
    }

    Vector3f pMaxSoma = _somaCenter + Vector3f(_somaMaxRadius);
    Vector3f pMinSoma = _somaCenter - Vector3f(_somaMaxRadius);

    if (pMaxSoma.x() > _pMax.x()) _pMax.x() = pMaxSoma.x();
    if (pMaxSoma.y() > _pMax.y()) _pMax.y() = pMaxSoma.y();
    if (pMaxSoma.z() > _pMax.z()) _pMax.z() = pMaxSoma.z();
    if (pMinSoma.x() < _pMin.x()) _pMin.x() = pMinSoma.x();
    if (pMinSoma.y() < _pMin.y()) _pMin.y() = pMinSoma.y();
    if (pMinSoma.z() < _pMin.z()) _pMin.z() = pMinSoma.z();

    _sections.clear();
    _sections = newSections;
    _firstSections.clear();
    _firstSections = newFirstSections;
}

Samples NeuronMorphology::getSomaSamples() const
{
    return _somaSamples;
}

Sections NeuronMorphology::getFirstSections() const
{
    return _firstSections;
}

Vector3f NeuronMorphology::getSomaCenter() const
{
    return _somaCenter;
}

float NeuronMorphology::getSomaMeanRadius() const
{
    return _somaMeanRadius;
}

float NeuronMorphology::getSomaMinRadius() const
{
    return _somaMinRadius;
}

float NeuronMorphology::getSomaMaxRadius() const
{
    return _somaMaxRadius;
}

// Reindexing of acyclic morphologies, with root nodes
void NeuronMorphology::reIndexMorphology()
{
    // Global indexing for the samples
    size_t sampleIndex = 0;

    // A collector for all the samples in the morphology including the ones that have been added
    // We will clear the current samples list, and then we reconstruct a new list of samples
    _samples.clear();
    _samples.shrink_to_fit();

    // Add the soma sample
    Sample* sample = new Sample(_somaCenter, _somaMeanRadius, PROCESS_TYPE::SOMA, sampleIndex++, -2);
    _samples.push_back(sample);

    // Get root sections, for neurons and astrocytes
    auto rootSections = getRootSections();

    // For each root section
    for (size_t i = 0; i < rootSections.size(); ++i)
    {
        // Get a reference to the root section
        auto& rootSection = rootSections[i];

        // Set the indices of the samples of the section
        auto rootSamples = rootSection->getSamples();
        for (size_t j = 0; j < rootSamples.size(); ++j)
        {
            rootSamples[j]->setIndex(sampleIndex++);
            auto parentIndex = (j == 0) ? 0 : sampleIndex - 1;
            rootSamples[j]->setParentIndex(parentIndex);

            // Add the sample to the samples list
            _samples.push_back(rootSamples[j]);
        }

        auto childrenIndices = rootSection->getChildrenIndices();
        for (size_t j = 0; j < childrenIndices.size(); ++j)
        {
            // Get a reference to the child section
            auto child = _sections[childrenIndices[j]];

            // Apply recursively
            child->reIndexSectionTree(_sections, sampleIndex, _samples);
        }
    }
}


void NeuronMorphology::_constructMorphologyFromSWC(const NeuronSWCSamples& swcSamples)
{
    // The _pMin and _pMax will be used to compute the bounding box of the neuron morphology
    _pMin = Vector3f(std::numeric_limits<float>::max());
    _pMax = Vector3f(std::numeric_limits<float>::lowest());

    // Sample index
    size_t index = 0;

    // Isolate the soma samples, id = 1
    std::vector< size_t > somaSamplesIndices;

    // SWC paths, where some could be sections and others could be concatinated lists of sections
    std::vector< std::vector< size_t > > swcPathsIndices;

    // Parse the list, and construct the paths
    while(true)
    {
        // Construct the path
        std::vector< size_t > swcPathIndices;
        while (true)
        {
            // If we have processed the last sample, then break
            if (index >= swcSamples.size() - 1)
                break;

            // Soma sample
            if (swcSamples[index]->type == PROCESS_TYPE::SOMA)
            {
                somaSamplesIndices.push_back(swcSamples[index]->id);
                index++;
                break;
            }
            else
            {
                // Get a reference to the sample
                auto swcSample = swcSamples[index];

                // If there is no increment by 1, then we have either a begining or end
                if (swcSample->parentId != swcSample->id - 1)
                {
                    // If the path is empty
                    if (swcPathIndices.empty())
                    {
                        if (swcSample->parentId != 1)
                        {
                            // Add the last sample of the parent section to be the first sample
                            auto parentSampleArrayIndex = swcSample->parentId;
                            swcPathIndices.push_back(swcSamples[parentSampleArrayIndex]->id);
                        }

                        // Add the first sample of the section to be the second sample
                        swcPathIndices.push_back(swcSample->id);

                        // Increment the index for the next sample
                        index++;
                    }
                    else
                    {
                        // It is the last sample of the section
                        swcPathsIndices.push_back(swcPathIndices);
                        swcPathIndices.clear();
                        swcPathIndices.shrink_to_fit();
                        break;
                    }
                }
                else
                {
                    // It is a middle section sample, the add it to the swcSection
                    swcPathIndices.push_back(swcSample->id);

                    // Increment the index for the next sample
                    index++;
                }
            }
        }

        // If we have processed the last sample, then break
        if (index >= swcSamples.size() - 1)
            break;
    }

    // List the first samples
    std::vector< size_t > firstSamplesIds;
    for (size_t i = 0; i < swcPathsIndices.size(); ++i)
    {
        firstSamplesIds.push_back(swcPathsIndices[i][0]);
        std::cout << firstSamplesIds[i] << " ";
    }
    std::cout << "\n";

    // The segments along each path
    std::vector< std::vector< size_t > > pathSegments;

    // Construct sections from the paths
    for (size_t i = 0; i < swcPathsIndices.size(); ++i)
    {
        // A new SWC section
        std::vector< size_t > segmentTerminals;

        // Get a reference to the current path
        auto pathIds = swcPathsIndices[i];

        for (size_t j = 0; j < pathIds.size(); ++j)
        {
            for (size_t k = 0; k < firstSamplesIds.size(); ++k)
            {
                if (pathIds[j] == firstSamplesIds[k])
                {
                    segmentTerminals.push_back(pathIds[j]);
                }
            }
        }

        // Add the last sample to the section
        segmentTerminals.push_back(pathIds.back());

        // Append
        pathSegments.push_back(segmentTerminals);
    }


    std::vector< std::pair< size_t, size_t> > sectionTerminals;

    for (size_t i = 0; i < pathSegments.size(); ++i)
    {
        for (size_t j = 0; j < pathSegments[i].size() - 1; ++j)
        {
            std::pair< size_t, size_t > terminals;
            terminals.first = pathSegments[i][j];
            terminals.second = pathSegments[i][j + 1];
            sectionTerminals.push_back(terminals);
        }
    }

    std::cout << sectionTerminals.size() << "\n";
    for (size_t i = 0; i < sectionTerminals.size(); i++)
        std::cout << sectionTerminals[i].first << " " << sectionTerminals[i].second << "\n";






    exit(0);

    // Construct the Ultraliser Sections and Samples lists from the swcSections list


}

//void NeuronMorphology::_constructMorphologyFromSWC_(const NeuronSWCSamples& swcSamples)
//{
//    // The _pMin and _pMax will be used to compute the bounding box of the neuron morphology
//    _pMin = Vector3f(std::numeric_limits<float>::max());
//    _pMax = Vector3f(std::numeric_limits<float>::lowest());

//    // Construct the samples and the sections list from the tree structure
//    std::stack< NeuronSWCSample* > samplesStack;
//    std::stack< Section* > sectionStack;
//    size_t sectionId = 0;

//    // The first sample is always the SOMA
//    samplesStack.push(swcSamples[0]);

//    // Initially, nothing is in the section stack
//    sectionStack.push(nullptr);

//    // Process the samples stack until empty where by then the sections are constructed
//    while (!samplesStack.empty())
//    {
//        // Gets the first element in the samples stack, and then removes it
//        auto swcSample = samplesStack.top();
//        samplesStack.pop();

//        // Get the first section
//        auto section = sectionStack.top();
//        sectionStack.pop();

//        Ultraliser::Vector3f position(swcSample->x, swcSample->y, swcSample->z);
//        float radius = swcSample->r;

//        Vector3f pMaxSample = position + Vector3f(radius);
//        Vector3f pMinSample = position - Vector3f(radius);

//        if (pMaxSample.x() > _pMax.x()) _pMax.x() = pMaxSample.x();
//        if (pMaxSample.y() > _pMax.y()) _pMax.y() = pMaxSample.y();
//        if (pMaxSample.z() > _pMax.z()) _pMax.z() = pMaxSample.z();

//        if (pMinSample.x() < _pMin.x()) _pMin.x() = pMinSample.x();
//        if (pMinSample.y() < _pMin.y()) _pMin.y() = pMinSample.y();
//        if (pMinSample.z() < _pMin.z()) _pMin.z() = pMinSample.z();

//        // Construct a new sample
//        Sample* sample = new Sample(position, radius,
//                                    swcSample->type, swcSample->id, swcSample->parentId);

//        // If this sample is somatic, either the soma average sample or a profile point
//        if (swcSample->type == PROCESS_TYPE::SOMA)
//        {
//            // _somaSamples.push_back(sample);
//            for (auto child : swcSample->childrenSamples)
//            {
//                samplesStack.push(child);
//                if (child->type == PROCESS_TYPE::SOMA)
//                {
//                    sectionStack.push(nullptr);
//                }
//                else
//                {
//                    auto newSection = new Section(sectionId, child->type);
//                    _sections.push_back(newSection);
//                    _firstSections.push_back(newSection);
//                    ++sectionId;
//                    sectionStack.push(newSection);
//                }
//            }
//        }

//        // Skeleton samples
//        else
//        {
//            // Add the sample to the skeleton samples
//            _samples.push_back(sample);

//            // Add the sample to the section
//            section->addSample(sample);

//            // Get the children samples
//            auto children = swcSample->childrenSamples;

//            // If there is only a single child
//            if (children.size() == 1)
//            {
//                samplesStack.push(children[0]);
//                sectionStack.push(section);
//            }

//            // Multiple children
//            else
//            {
//                for (auto child : children)
//                {
//                    samplesStack.push(child);
//                    section->addChildIndex(sectionId);

//                    // Construct a new section corresponding to the child
//                    auto newSection = new Section(sectionId, child->type);

//                    // Add it to the list of sections
//                    _sections.push_back(newSection);

//                    // Update the counters to keep track
//                    ++sectionId;

//                    // Parent-child link
//                    newSection->addParentIndex(section->getIndex());

//                    // Add the sample to the new section
//                    newSection->addSample(sample);

//                    // Add the new section to the stack
//                    sectionStack.push(newSection);
//                }
//            }
//        }
//    }

//    // Get the somatic samples from the initial sections of all the neurites
//    for (const auto& section: _firstSections)
//    {
//        const auto sample = section->getFirstSample();
//        _somaSamples.push_back(sample);
//    }

//    // Initially, the soma center is set to Zero.
//    _somaCenter = Vector3f(0.0f);
//    _somaMeanRadius = 0.0f;
//    _somaMinRadius = 1e10;
//    _somaMaxRadius = -1e10;

//    if (_somaSamples.size() > 1)
//    {
//        // Compute the soma center
//        for (auto sample : _somaSamples)
//        {
//            _somaCenter += sample->getPosition();
//        }
//        _somaCenter /= _somaSamples.size();

//        // Compute the minimum, average and maximum radii
//        for (auto sample : _somaSamples)
//        {
//            const auto radius = (sample->getPosition() - _somaCenter).abs();

//            if (radius < _somaMinRadius)
//                _somaMinRadius = radius;

//            if (radius > _somaMaxRadius)
//                _somaMaxRadius = radius;

//            _somaMeanRadius += radius;
//        }

//        // Compute the minimum, average and maximum radii
//        _somaMeanRadius /= _somaSamples.size();
//    }
//    else
//    {
//        _somaCenter = _somaSamples[0]->getPosition();
//        _somaMeanRadius = _somaSamples[0]->getRadius();
//        _somaMinRadius = _somaMeanRadius;
//        _somaMaxRadius = _somaMeanRadius;
//    }

//    Vector3f pMaxSoma = _somaCenter + Vector3f(_somaMaxRadius);
//    Vector3f pMinSoma = _somaCenter - Vector3f(_somaMaxRadius);

//    if (pMaxSoma.x() > _pMax.x()) _pMax.x() = pMaxSoma.x();
//    if (pMaxSoma.y() > _pMax.y()) _pMax.y() = pMaxSoma.y();
//    if (pMaxSoma.z() > _pMax.z()) _pMax.z() = pMaxSoma.z();
//    if (pMinSoma.x() < _pMin.x()) _pMin.x() = pMinSoma.x();
//    if (pMinSoma.y() < _pMin.y()) _pMin.y() = pMinSoma.y();
//    if (pMinSoma.z() < _pMin.z()) _pMin.z() = pMinSoma.z();





//    std::cout << "Reading and validating..... \n\n";
//    for (size_t ii = 0; ii < _samples.size(); ++ii)
//    {
//        auto sample = _samples[_samples.size() - ii - 1];
//        std::cout   << sample->getIndex() + 1 << SPACE
//                    << mapNeuronProcessTypeToSWCIndex(sample->getType()) << SPACE
//                    << sample->getPosition().x() << SPACE
//                    << sample->getPosition().y() << SPACE
//                    << sample->getPosition().z() << SPACE
//                    << sample->getRadius()       << SPACE
//                    << sample->getParentIndex() + 1 << NEW_LINE;
//    }
//}

void NeuronMorphology::_constructMorphologyFromH5(const H5Samples& h5Samples,
                                                  const H5Sections& h5Sections)
{
    // The _pMin and _pMax will be used to compute the bounding box of the neuron morphology
    _pMin = Vector3f(std::numeric_limits<float>::max());
    _pMax = Vector3f(std::numeric_limits<float>::lowest());

    // Compute the boundinx box from the H5 samples
    for (auto h5Sample : h5Samples)
    {
        Ultraliser::Vector3f position(h5Sample.x, h5Sample.y, h5Sample.z);

        Vector3f pMaxSample = position + Vector3f(h5Sample.r * 0.5f);
        Vector3f pMinSample = position - Vector3f(h5Sample.r * 0.5f);

        if (pMaxSample.x() > _pMax.x()) _pMax.x() = pMaxSample.x();
        if (pMaxSample.y() > _pMax.y()) _pMax.y() = pMaxSample.y();
        if (pMaxSample.z() > _pMax.z()) _pMax.z() = pMaxSample.z();

        if (pMinSample.x() < _pMin.x()) _pMin.x() = pMinSample.x();
        if (pMinSample.y() < _pMin.y()) _pMin.y() = pMinSample.y();
        if (pMinSample.z() < _pMin.z()) _pMin.z() = pMinSample.z();
    }

    // Create the soma section, for indexing only
    Section* somaSection = new Section(0, PROCESS_TYPE::SOMA);
    _sections.push_back(somaSection);

    // The soma is located at index 0, but we will consider it a section for indexing!
    for (size_t i = 1; i < h5Sections.size() - 1; ++i)
    {
        // Current section index
        const auto sectionIndex = i;

        // First point index
        const auto firstPointIdx = h5Sections[i].offsetIndex;

        // Last point index
        const auto lastPointIdx = h5Sections[i + 1].offsetIndex - 1;

        // Get the process type
        auto processType = mapNeuronH5IndexToType(h5Sections[i].sectionType);

        // Create a new morphology section
        Section* section = new Section(sectionIndex, processType);

        // Construct a list of samples
        Samples samples;
        for (size_t s = firstPointIdx; s <= lastPointIdx; s++)
        {
            const auto sampleIndex = s;
            size_t parentIndex;

            // Create the sample
            Sample* sample =
                    new Sample(Ultraliser::Vector3f(h5Samples[s].x, h5Samples[s].y, h5Samples[s].z),
                               h5Samples[s].r * 0.5f, processType, s, parentIndex);

            // Update the samples list
            section->addSample(sample);

            _samples.push_back(sample);
        }

        // Update the section parent
        section->addParentIndex(h5Sections[i].parentIndex);

        // Add the section to the list
        _sections.push_back(section);

        // Determine if this is a first section or not!
        if (h5Sections[i].parentIndex == 0)
        {
            _firstSections.push_back(section);
        }
    }

    // Build the tree (add the children indices) from the linear list
    for (size_t i = 1; i < _sections.size(); ++i)
    {
        // Get parent index
        size_t parentSectionIndex = _sections[i]->getParentIndices()[0];

        // Update the choldren list
        _sections[parentSectionIndex]->addChildIndex(_sections[i]->getIndex());
    }

    // Get the somatic samples from the initial sections of all the neurites
    for (const auto& section: _firstSections)
    {
        const auto sample = section->getFirstSample();
        _somaSamples.push_back(sample);
    }

    // Initially, the soma center is set to Zero.
    _somaCenter = Vector3f(0.0f);
    _somaMeanRadius = 0.0f;
    _somaMinRadius = 1e10;
    _somaMaxRadius = -1e10;

    if (_somaSamples.size() > 1)
    {
        // Compute the soma center
        for (auto sample : _somaSamples)
        {
            _somaCenter += sample->getPosition();
        }
        _somaCenter /= _somaSamples.size();

        // Compute the minimum, average and maximum radii
        for (auto sample : _somaSamples)
        {
            const auto radius = (sample->getPosition() - _somaCenter).abs();

            if (radius < _somaMinRadius)
                _somaMinRadius = radius;

            if (radius > _somaMaxRadius)
                _somaMaxRadius = radius;

            _somaMeanRadius += radius;
        }

        // Compute the minimum, average and maximum radii
        _somaMeanRadius /= _somaSamples.size();
    }
    else
    {
        _somaCenter = _somaSamples[0]->getPosition();
        _somaMeanRadius = _somaSamples[0]->getRadius();
        _somaMinRadius = _somaMeanRadius;
        _somaMaxRadius = _somaMeanRadius;
    }
}

NeuronMorphology* readNeuronMorphology(std::string& morphologyPath)
{
    if (String::subStringFound(morphologyPath, std::string(".swc")) ||
        String::subStringFound(morphologyPath, std::string(".SWC")) ||
        String::subStringFound(morphologyPath, std::string(".Swc")))
    {
        // Read the file
        auto reader = std::make_unique< NeuronSWCReader >(morphologyPath);

        // Get a pointer to the morphology to start using it
        return reader->getMorphology();
    }
    else if (String::subStringFound(morphologyPath, std::string(".h5")) ||
             String::subStringFound(morphologyPath, std::string(".H5")))
    {
#ifdef ULTRALISER_USE_H5
        // Read the file
        auto reader = std::make_unique< NeuronH5Reader >(morphologyPath);

        // Get a pointer to the morphology to start using it
        return reader->getMorphology();
#else
        LOG_ERROR("Ultraliser compiled with NO support to read .H5 morphologies!");
        return nullptr;
#endif
    }
    else
    {
        LOG_ERROR("Unrecognized morphology file format [ %s ]", morphologyPath.c_str());
    }

    // To avoid any warning issues.
    return nullptr;
}

}
