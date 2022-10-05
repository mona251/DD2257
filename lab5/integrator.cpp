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
// Access the vector field with vectorField.interpolate(...)

dvec2 Integrator::Euler(const VectorField2& vectorField, const dvec2& position, double step)
{
    dvec2 v = vectorField.interpolate(position);
    return position + v * step; // SImple Euler
}

dvec2 Integrator::RK4(const VectorField2& vectorField, const dvec2& position, double step,
                      bool backwards, bool normalized) {
    if (backwards) {
        step = -step; // If backwards we reverse the step direction
    }
    // Runge Kutta 4th Order according to Formula
    // Normalize vectorfield if directional
    dvec2 v1 = vectorField.interpolate(position); 
    v1 = normalized && glm::length(v1) > 0 ? glm::normalize(v1) : v1;
    dvec2 v2 = vectorField.interpolate(position + 0.5 * step * v1);
    v2 = normalized && glm::length(v2) > 0 ? glm::normalize(v2) : v2;
    dvec2 v3 = vectorField.interpolate(position + 0.5 * step * v2);
    v3 = normalized && glm::length(v3) > 0 ? glm::normalize(v3) : v3;
    dvec2 v4 = vectorField.interpolate(position + step * v3);
    v4 = normalized && glm::length(v4) > 0 ? glm::normalize(v4) : v4;
    return position + step * (v1 / 6.0 + v2 / 3.0 + v3 / 3.0 + v4 / 6.0);
}

std::vector<dvec2> Integrator::Streamline(const VectorField2& vectorField, const dvec2& position,
    double step, int steps){
    
    std::vector<dvec2> streamline;
    dvec2 startPos = position;
    dvec2 currentPos = position;
    dvec2 prevPos = position;
    bool halt = false;
    
    //Backward pass
    for (int s = 0; s < steps; s++) {
        currentPos = RK4(vectorField, prevPos, step, true, true);
        
        if (!vectorField.isInside(currentPos)) {
            currentPos = vectorField.clampPositionToBBox(currentPos);
            halt = true;
        }
        if (glm::length(vectorField.interpolate(currentPos)) < 0.000001) {
            halt = true;
        }
        if(glm::length(prevPos-currentPos) < 0.5*step){
            halt = true;
        }
        if(halt){
            break;
        }
        else {
            streamline.push_back(currentPos);
        }
        prevPos = currentPos;
    }
    //Reverses the elements so they are in the correct order
    if(streamline.size() >= 2){
        std::reverse(streamline.begin(), streamline.end());
    }
    //Add middle element and prepare forward
    streamline.push_back(startPos);
    currentPos = startPos;
    prevPos = startPos;
    halt = false;
    //Forward pass
    for (int s = 0; s < steps; s++) {
        currentPos = RK4(vectorField, prevPos, step, false, true);
        
        if (!vectorField.isInside(currentPos)) {
            currentPos = vectorField.clampPositionToBBox(currentPos);
            halt = true;
        }
        if (glm::length(vectorField.interpolate(currentPos)) < 0.000001) {
            halt = true;
        }
        if(glm::length(prevPos-currentPos) < 0.5*step){
            //Wow there's this really cool feature where you can bounce over some 
            //extremities and essentially never reach the other stop conditions when 
            //integrating over the directional field I sure do love hunting for bugs :)
            //This can still happen but is much less likely now
            halt = true;
        }
        if(halt){
            break;
        }
        else {
            streamline.push_back(currentPos);
        }
        prevPos = currentPos;
    }
    return streamline;
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
