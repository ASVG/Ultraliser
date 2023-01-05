/***************************************************************************************************
 * Copyright (c) 2016 - 2021
 * Blue Brain Project (BBP) / Ecole Polytechnique Federale de Lausanne (EPFL)
 *
 * Author(s)
 *      Nadir Roman Gerrero < nadir.romanguerrero@epfl.ch >
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

#include "Mesh.h"
#include "MeshOperations.h"

#include <utilities/Utilities.h>

namespace
{
inline static const size_t nullCollapse = std::numeric_limits<size_t>::max();

/**
 * @brief Triangle data for edge-collapse operation
 */
struct CollapseTriangle
{
    size_t index;
    Ultraliser::Vec3ui_64 vertices;
    Ultraliser::Vector3f normal;

    bool containsVertex(size_t index) const
    {
        return vertices.x() == index || vertices.y() == index || vertices.z() == index;
    }

    void replaceVertex(size_t containing, size_t neww)
    {
        assert(containsVertex(containing));
        assert(!containsVertex(neww));
        for(size_t i = 0; i < 3; ++i)
        {
            if(vertices[i] != containing)
            {
                continue;
            }
            vertices[i] = neww;
            return;
        }
    }

    operator bool() const noexcept
    {
        return index != nullCollapse;
    }
};

/**
 * @brief Generates a list of CollapseTriangle from the data in a Ultraliser::Mesh object
 */
class CollapseTriangleFactory
{
public:
    static std::vector<CollapseTriangle> create(const Ultraliser::Mesh &mesh)
    {
        auto vertices = mesh.getVertices();
        auto triangles = mesh.getTriangles();
        auto triangleCount = mesh.getNumberTriangles();

        auto result = std::vector<CollapseTriangle>();
        result.reserve(triangleCount);

        for(size_t i = 0; i < triangleCount; ++i)
        {
            result.push_back({i, _vec3iToVec3ui(triangles[i]), _computeNormal(triangles[i], vertices)});
        }

        return result;
    }
private:
    static Ultraliser::Vec3ui_64 _vec3iToVec3ui(const Ultraliser::Vec3i_64 &face)
    {
        return {I2UI64(face.x()), I2UI64(face.y()), I2UI64(face.z())};
    }

    static Ultraliser::Vec3i_64 _vec3uiToVec3i(const Ultraliser::Vec3ui_64 &face)
    {
        return {UI2I64(face.x()), UI2I64(face.y()), UI2I64(face.z())};
    }

    static Ultraliser::Vector3f _computeNormal(const Ultraliser::Vec3i_64 &face, const Ultraliser::Vertex *vertices)
    {
        auto &v1 = vertices[face.x()];
        auto &v2 = vertices[face.y()];
        auto &v3 = vertices[face.z()];
        return Ultraliser::computeNormal(v1, v2, v3);
    }
};

/**
 * @brief Vertex data for edge-collapse operation
 */
struct CollapseVertex
{
    size_t index;
    Ultraliser::Vector3f position;
    std::unordered_set<size_t> triangles;
    std::unordered_set<size_t> neighbours;
    float cost = std::numeric_limits<float>::max();
    size_t collapseVertex = nullCollapse;

    void setCollapseInfo(float cost, size_t collapsee)
    {
        this->cost = cost;
        this->collapseVertex = collapsee;
    }

    void removeNeighbour(const CollapseVertex &other)
    {
        neighbours.erase(other.index);
    }

    void addNeighbour(const CollapseVertex &other)
    {
        neighbours.insert(other.index);
    }

    bool isNeighbour(const CollapseVertex &other)
    {
        return neighbours.find(other.index) != neighbours.end();
    }

    void removeTriangle(size_t triangleIndex)
    {
        assert(triangles.find(triangleIndex) != triangles.end());
        triangles.erase(triangleIndex);
    }

    void addTriangle(size_t triangleIndex)
    {
        assert(triangles.find(triangleIndex) == triangles.end());
        triangles.insert(triangleIndex);
    }

    operator bool() const noexcept
    {
        return index != nullCollapse;
    }
};

/**
 * @brief Generates a list of CollapseVertex from the data in a Ultraliser::Mesh object
 */
class CollapseVertexFactory
{
public:
    static std::vector<CollapseVertex> create(const Ultraliser::Mesh &mesh)
    {
        auto vertices = mesh.getVertices();
        auto vertexCount = mesh.getNumberVertices();
        auto triangles = mesh.getTriangles();
        auto triangleCount = mesh.getNumberTriangles();

        auto result = std::vector<CollapseVertex>();
        result.reserve(vertexCount);

        auto vertexTriangleMap = _createVertexTriangleMap(mesh);

        for(size_t i = 0; i < vertexCount; ++i)
        {
            auto &vertexTriangles = vertexTriangleMap[i];
            auto vertexNeighbour = _vertexNeighbours(i, triangles, vertexTriangles);
            result.push_back({i, vertices[i], std::move(vertexTriangles), std::move(vertexNeighbour)});
        }

        return result;
    }
private:
    static std::vector<std::unordered_set<size_t>> _createVertexTriangleMap(const Ultraliser::Mesh &mesh)
    {
        auto triangles = mesh.getTriangles();
        auto triangleCount = mesh.getNumberTriangles();
        auto vertexCount = mesh.getNumberVertices();

        auto result = std::vector<std::unordered_set<size_t>>(vertexCount);
        for(size_t i = 0; i < triangleCount; ++i)
        {
            result[triangles[i].x()].insert(i);
            result[triangles[i].y()].insert(i);
            result[triangles[i].z()].insert(i);
        }

        return result;
    }

    static std::unordered_set<size_t> _vertexNeighbours(
        size_t vertexIndex, 
        const Ultraliser::Triangle *triangles, 
        const std::unordered_set<size_t> &vertexTriangles)
    {
        auto result = std::unordered_set<size_t>();
        for(auto vertexTriangle : vertexTriangles)
        {
            auto &triangle = triangles[vertexTriangle];
            for(size_t i = 0; i < 3; ++i)
            {
                if(triangle[i] == vertexIndex)
                {
                    continue;
                }
                result.insert(triangle[i]);
            }
        }
        return result;
    }
};

/**
 * @brief Holds and gives access to read/modify the collapse mesh data (vertices and triangles).
 */
class CollapseMeshData
{
public:
    CollapseMeshData(const Ultraliser::Mesh &mesh)
     : _triangles(CollapseTriangleFactory::create(mesh))
     , _vertices(CollapseVertexFactory::create(mesh))
    {
    }

    bool validVertexCount() const noexcept
    {
        size_t count = 0;
        for(auto &vertex : _vertices)
        {
            count += vertex? 1 : 0;
        }
        return count;
    }

    CollapseVertex &findMinimumCostVertex()
    {
        size_t minimum = nullCollapse;
        for(size_t i = 1; i < _vertices.size(); ++i)
        {
            if(!_vertices[i])
            {
                continue;
            }

            if(minimum == nullCollapse || _vertices[i].cost < _vertices[minimum].cost)
            {
                minimum = i;
            }
        }
        return _vertices[minimum];
    }

    void replaceVertex(CollapseTriangle &triangle, CollapseVertex &u, CollapseVertex &v)
    {
        assert(u && v);
        triangle.replaceVertex(u.index, v.index);

        u.removeTriangle(triangle.index);
        v.addTriangle(triangle.index);

        for(size_t i = 0; i < 3; ++i)
        {
            auto vertexIndex = triangle.vertices[i];
            auto &oldNeighbour = _vertices[vertexIndex];
            _removeIfNotNeighbour(u, oldNeighbour);
            _removeIfNotNeighbour(oldNeighbour, u);
        }

        for(size_t i = 0; i < 3; ++i)
        {
            auto &vertex = _vertices[triangle.vertices[i]];
            for(size_t j = 0; j < 3; ++j)
            {
                if(i == j)
                {
                    continue;
                }
                vertex.addNeighbour(_vertices[triangle.vertices[j]]);
            }
        }

        _computeTriangleNormal(triangle);
    }

    void destroyVertex(size_t index)
    {
        assert(index < _vertices.size());
        auto &vertex = _vertices[index];
        destroyVertex(vertex);
    }

    void destroyVertex(CollapseVertex &vertex)
    {
        assert(static_cast<bool>(vertex));
        assert(vertex.triangles.empty());
        for(auto neighbourIndex : vertex.neighbours)
        {
            auto &neighbourVertex = _vertices[neighbourIndex];
            neighbourVertex.removeNeighbour(vertex);
        }

        vertex.neighbours.clear();
        vertex.index = nullCollapse;
    }

    void destroyTriangle(size_t index)
    {
        assert(index < _triangles.size());
        auto &triangle = _triangles[index];
        destroyTriangle(triangle);
    }

    void destroyTriangle(CollapseTriangle &triangle)
    {
        assert(triangle);
        for(size_t i = 0; i < 3; ++i)
        {
            auto &vertex = _vertices[triangle.vertices[i]];
            vertex.triangles.erase(triangle.index);

            for(size_t j = 0; j < 3; ++j)
            {
                if(i == j)
                {
                    continue;
                }
                auto &vertex2 = _vertices[triangle.vertices[j]];
                _removeIfNotNeighbour(vertex, vertex2);
            }
        }
        triangle.index = nullCollapse;
    }

    size_t getVertexCount() const noexcept
    {
        return _vertices.size();
    }

    CollapseTriangle &getTriangle(size_t index) noexcept
    {
        assert(index < _triangles.size());
        return _triangles[index];
    }

    CollapseVertex &getVertex(size_t index) noexcept
    {
        assert(index < _vertices.size());
        return _vertices[index];
    }

    const std::vector<CollapseTriangle> &getTriangles() const noexcept
    {
        return _triangles;
    }

    const std::vector<CollapseVertex> &getVertices() const noexcept
    {
        return _vertices;
    }

private:
    void _removeIfNotNeighbour(CollapseVertex &vertex, CollapseVertex &removal)
    {
        if(!vertex.isNeighbour(removal))
        {
            return;
        }

        for(auto triangleIndex : vertex.triangles)
        {
            auto &triangle = _triangles[triangleIndex];
            if(triangle.containsVertex(removal.index))
            {
                return;
            }
        }

        vertex.removeNeighbour(removal);
    }

    void _computeTriangleNormal(CollapseTriangle &triangle)
    {
        auto &p1 = _vertices[triangle.vertices.x()].position;
        auto &p2 = _vertices[triangle.vertices.y()].position;
        auto &p3 = _vertices[triangle.vertices.z()].position;
        triangle.normal = Ultraliser::computeNormal(p1, p2, p3);
    }

private:
    std::vector<CollapseTriangle> _triangles;
    std::vector<CollapseVertex> _vertices;
};

/**
 * @brief Implements the edge collapse cost algorithm
 * From https://github.com/andandandand/progressive-mesh-reduction-with-edge-collapse
 */
class CollapseEdgeCostAlgorithm
{
public:
    CollapseEdgeCostAlgorithm(CollapseMeshData &mesh)
     : _mesh(mesh)
    {
        _updateMeshCost();
    }

    void updateVerticesCost(const std::unordered_set<size_t> &vertices)
    {
        for(auto vertexIndex : vertices)
        {
            _updateVertexCost(vertexIndex);
        }
    }

private:
    void _updateMeshCost()
    {
        for(size_t i = 0; i < _mesh.getVertexCount(); ++i)
        {
            _updateVertexCost(i);
        }
    }

    void _updateVertexCost(size_t index)
    {
        auto &vertex = _mesh.getVertex(index);
        if(!vertex)
        {
            return;
        }

        if(vertex.neighbours.empty())
        {
            vertex.setCollapseInfo(-1.f, nullCollapse);
            return;
        }

        vertex.setCollapseInfo(std::numeric_limits<float>::max(), nullCollapse);
        for(auto neighbourIndex : vertex.neighbours)
        {
            auto &neighbour = _mesh.getVertex(neighbourIndex);
            if(!neighbour)
            {
                continue;
            }

            auto edgeCost = _computeEdgeCost(vertex, neighbour);
            if(edgeCost > vertex.cost)
            {
                continue;
            }

            vertex.setCollapseInfo(edgeCost, neighbourIndex);
        }
    }

    float _computeEdgeCost(const CollapseVertex &u, const CollapseVertex &v)
    {
        auto edgeLen = (u.position - v.position).abs();
        auto curvature = _findMaxCurvature(u, v);
        return edgeLen * curvature;
    }

    std::vector<size_t> _findEdgeTriangles(const CollapseVertex &u, const CollapseVertex &v)
    {
        auto result = std::vector<size_t>();
        for(auto triangleIndex : u.triangles)
        {
            auto &triangle = _mesh.getTriangle(triangleIndex);
            if(!triangle.containsVertex(v.index))
            {
                continue;
            }
            result.push_back(triangleIndex);
        }
        return result;
    }

    float _findMaxCurvature(const CollapseVertex &u, const CollapseVertex &v)
    {
        auto sides = _findEdgeTriangles(u, v);
        auto curvature = 0.f;
        
        for(auto triangleIndex : u.triangles)
        {
            auto localCurvature = 1.f;
            auto &triangle = _mesh.getTriangle(triangleIndex);

            for(auto sideTriangleIndex : sides)
            {   
                auto &sideTriangle = _mesh.getTriangle(sideTriangleIndex);
                auto triangleCurvature = _trianglesCurvature(triangle, sideTriangle); 
                localCurvature = std::min(localCurvature, (1.f - triangleCurvature) * 0.5f);
            }
            curvature = std::max(curvature, localCurvature);
        }
        return curvature;
    }

    float _trianglesCurvature(CollapseTriangle &t1, CollapseTriangle &t2)
    {
        auto &t1Normal = t1.normal;
        auto &t2Normal = t2.normal;
        return Ultraliser::Vector3f::dot(t1Normal, t2Normal);
    }

private:
    CollapseMeshData &_mesh;
};

/**
 * @brief Implements the edge collapse algorithm
 * From https://github.com/andandandand/progressive-mesh-reduction-with-edge-collapse
 */
class CollapseEdgeAlgorithm
{
public:
    CollapseEdgeAlgorithm(CollapseMeshData &mesh)
     : _mesh(mesh)
    {
    }

    void collapseVertex(CollapseVertex &u)
    {
        assert(u.collapseVertex != nullCollapse);
        auto &v = _mesh.getVertex(u.collapseVertex);
        if(!v)
        {
            _mesh.destroyVertex(u);
            return;
        }

        _destroyCommonTriangles(u, v);
        _replaceVertex(u, v);
        _mesh.destroyVertex(u);
    }

private:
    void _destroyCommonTriangles(CollapseVertex &u, CollapseVertex &v)
    {
        auto trianglesToDelete = std::vector<size_t>();
        trianglesToDelete.reserve(u.triangles.size());

        for(auto triangleIndex : u.triangles)
        {
            auto &triangle = _mesh.getTriangle(triangleIndex);
            if(!triangle.containsVertex(v.index))
            {
                continue;
            }
            trianglesToDelete.push_back(triangleIndex);
        }
        for(auto triangleToDelete : trianglesToDelete)
        {
            _mesh.destroyTriangle(triangleToDelete);
        }
    }

    void _replaceVertex(CollapseVertex &u, CollapseVertex &v)
    {
        auto triangleCopy = u.triangles;
        for(auto triangleIndex : triangleCopy)
        {
            auto &triangle = _mesh.getTriangle(triangleIndex);
            _mesh.replaceVertex(triangle, u, v);
        }
    }

private:
    CollapseMeshData &_mesh;
};

/**
 * @brief Converts a CollapseMeshData object into an list of vertices and triangles for Ultraliser::Mesh
 * From https://github.com/andandandand/progressive-mesh-reduction-with-edge-collapse
 */
class CollapseMeshConverter
{
public:
    CollapseMeshConverter(const CollapseMeshData &mesh)
    {
        auto vacancyMap = _generateVacanceyMap(mesh);
        auto conversionMap = _generateConversionMap(vacancyMap);
        _buildVertexList(vacancyMap, mesh);
        _buildTriangleList(conversionMap, mesh);
    }

    Ultraliser::Triangles triangles;
    Ultraliser::Vertices vertices;

private:
    std::vector<bool> _generateVacanceyMap(const CollapseMeshData &mesh)
    {
        auto &triangles = mesh.getTriangles();
        auto &vertices = mesh.getVertices();
        auto result = std::vector<bool>(vertices.size(), false);
        for(auto &triangle : triangles)
        {
            if(!triangle)
            {
                continue;
            }

            result[triangle.vertices.x()] = true;
            result[triangle.vertices.y()] = true;
            result[triangle.vertices.z()] = true;
        }
        return result;
    }

    std::vector<size_t> _generateConversionMap(const std::vector<bool> &vacancy)
    {
        auto result = std::vector<size_t>(vacancy.size(), 0);
        auto offset = 0ul;
        for(size_t i = 0; i < vacancy.size(); ++i)
        {
            if(!vacancy[i])
            {
                continue;
            }
            result[i] = offset++;
        }
        return result;
    }

    size_t _countValidVertices(const std::vector<bool> &vacancy)
    {
        size_t count = 0;
        for(auto flag : vacancy)
        {
            count += flag? 1 : 0;
        }
        return count;
    }

    void _buildVertexList(const std::vector<bool> &vacancy, const CollapseMeshData &mesh)
    {
        auto &srcVertices = mesh.getVertices();
        auto validVertexCount = _countValidVertices(vacancy);
        vertices.reserve(validVertexCount);

        for(size_t i = 0; i < vacancy.size(); ++i)
        {
            if(vacancy[i])
            {
                vertices.push_back(srcVertices[i].position);
            }
        }
    }

    size_t _countValidTriangles(const std::vector<CollapseTriangle> &triangles)
    {
        size_t count = 0;
        for(auto &triangle : triangles)
        {
            count += static_cast<bool>(triangle)? 1 : 0;
        }
        return count;
    }

    void _buildTriangleList(const std::vector<size_t> conversionMap, const CollapseMeshData &mesh)
    {
        auto &srcTriangles = mesh.getTriangles();
        auto validTriangleCount = _countValidTriangles(srcTriangles);
        triangles.reserve(validTriangleCount);

        for(auto &srcTriangle : srcTriangles)
        {
            if(!srcTriangle)
            {
                continue;
            }
            auto &vertices = srcTriangle.vertices;

            auto x = UI2I64(conversionMap[vertices.x()]);
            auto y = UI2I64(conversionMap[vertices.y()]);
            auto z = UI2I64(conversionMap[vertices.z()]);
            triangles.emplace_back(x, y, z);
        }
    }
};

class IterationsCalculator
{
public:
    static size_t fromPercentage(const Ultraliser::Mesh &mesh, float percentage)
    {
        if(percentage <= 0.f)
        {
            throw std::invalid_argument("Cannot collapse the mesh to zero or negative percentage");
        }

        auto numVertices = mesh.getNumberVertices();
        return static_cast<size_t>(static_cast<double>(percentage) * numVertices);
    }
};
}

namespace Ultraliser
{
void Mesh::collapseEdges(float maxPercentage)
{
    if(maxPercentage >= 1.f)
    {
        return;
    }

    TIMER_SET;

    auto numIterations = IterationsCalculator::fromPercentage(*this, maxPercentage);
    auto collapseMesh = CollapseMeshData(*this);
    auto collapseCost = CollapseEdgeCostAlgorithm(collapseMesh);
    auto collapseAlgorithm = CollapseEdgeAlgorithm(collapseMesh);

    for(size_t i = 0; i < numIterations; ++i)
    {
        if(collapseMesh.validVertexCount() == 0)
        {
            LOG_STATUS("Stopped at ", i, " out of ", numIterations, " iterations");
            break;
        }

        auto &collapsableVertex = collapseMesh.findMinimumCostVertex();
        auto neighbourCopy = collapsableVertex.neighbours;

        collapseAlgorithm.collapseVertex(collapsableVertex);
        collapseCost.updateVerticesCost(neighbourCopy);
    }

    auto converter = CollapseMeshConverter(collapseMesh);
    _initFromVertexAndTriangleList(std::move(converter.vertices), std::move(converter.triangles));

    auto seconds = GET_TIME_SECONDS;
    LOG_STATUS("Time: ", numIterations, " edge-collapse iterations in ", seconds);
}
}