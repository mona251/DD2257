/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Wednesday, September 20, 2017 - 12:04:15
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labstreamlines/integrator.h>

namespace inviwo {

// TODO: Implement a single integration step here

 dvec2 Integrator::Euler(const VectorField2& vectorField, const dvec2& position, double scalar) {


     //Access the vector field with vectorField.interpolate(...)
     
     auto vf = vectorField.interpolate(position); //vf as in Vectorfield

     auto next_position = position + scalar * vf;
     

     return next_position;

 }

 dvec2 Integrator::RK4(const VectorField2& vectorField, const dvec2& position, double scalar, bool backwards, bool normalized) {

    auto vf1 = vectorField.interpolate(position); //v1
    if(normalized){
        vf1 = glm::length(vf1)== 0 ? vf1 : glm::normalize(vf1);
    }
    if(backwards)
        vf1 *= -1;
    auto vf2 = vectorField.interpolate(position + (scalar / 2) * vf1); //v2
    if(normalized){
        vf2 = glm::length(vf2)== 0 ? vf2 : glm::normalize(vf2);
    }
    if(backwards)
        vf2 *= -1;
    auto vf3 = vectorField.interpolate(position + (scalar / 2) * vf2); //v3
    if(normalized){
        vf3 = glm::length(vf3)== 0 ? vf3 : glm::normalize(vf3);
    }
    if(backwards)
        vf3 *= -1;
    auto vf4 = vectorField.interpolate(position + scalar * vf3); //vf4
    if(normalized){
        vf4 = glm::length(vf4)== 0 ? vf4 : glm::normalize(vf4);
    }
    if(backwards)
        vf4 *= -1;

    auto next_position = position + scalar * ((vf1 * 0.16666666666666666666666666666667) +
                                              (vf2 * 0.33333333333333333333333333333333) +
                                              (vf3 * 0.33333333333333333333333333333333) +
                                              (vf4 * 0.16666666666666666666666666666667));

    return next_position;
}

double Integrator::lengthVec2(const dvec2 vec){
    return pow(vec[0]*vec[0] + vec[1]*vec[1],0.5);
}

void Integrator::drawPoint(const dvec2& p, const vec4& color, IndexBufferRAM* indexBuffer,
                           std::vector<BasicMesh::Vertex>& vertices) {
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(p[0], p[1], 0), vec3(0, 0, 1), vec3(p[0], p[1], 0), color});
}

// Alias for draw point
void Integrator::drawNextPointInPolyline(const dvec2& p, const vec4& color,
                                         IndexBufferRAM* indexBuffer,
                                         std::vector<BasicMesh::Vertex>& vertices) {
    Integrator::drawPoint(p, color, indexBuffer, vertices);
}

void Integrator::drawLineSegment(const dvec2& v1, const dvec2& v2, const vec4& color,
                                 IndexBufferRAM* indexBuffer,
                                 std::vector<BasicMesh::Vertex>& vertices) {
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v1[0], v1[1], 0), vec3(0, 0, 1), vec3(v1[0], v1[1], 0), color});
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v2[0], v2[1], 0), vec3(0, 0, 1), vec3(v2[0], v2[1], 0), color});
}

}  // namespace inviwo
