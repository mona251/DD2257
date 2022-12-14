/*********************************************************************
 *  Author  : Himangshu Saikia, Wiebke Koepp
 *  Init    : Tuesday, September 19, 2017 - 15:08:24
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/algorithm/boundingbox.h>
#include <inviwo/core/interaction/events/mouseevent.h>
#include <labstreamlines/eulerrk4comparison.h>
#include <labstreamlines/integrator.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo EulerRK4Comparison::processorInfo_{
    "org.inviwo.EulerRK4Comparison",  // Class identifier
    "Euler RK4 Comparison",           // Display name
    "KTH Lab",                        // Category
    CodeState::Experimental,          // Code state
    Tags::None,                       // Tags
};

const ProcessorInfo EulerRK4Comparison::getProcessorInfo() const { return processorInfo_; }

EulerRK4Comparison::EulerRK4Comparison()
    : Processor()
    , inData("inData")
    , meshOut("meshOut")
    , meshBBoxOut("meshBBoxOut")
    , propStartPoint("startPoint", "Start Point", vec2(0.5, 0.5), vec2(0), vec2(1024), vec2(0.5))
    , mouseMoveStart(
          "mouseMoveStart", "Move Start", [this](Event* e) { eventMoveStart(e); },
          MouseButton::Left, MouseState::Press | MouseState::Move)
    , stepSizeEuler("stepSizeEuler", "Step size for Euler")
    , integrationStepsEuler("integrationStepsEuler", "integration steps for Euler", 0, 0, 100, 1)
    , stepSizeRK4("stepSizeRK4", "Step size for RK4")
    , integrationStepsRK4("integrationStepsRK4", "integration steps for RK4", 0, 0, 100, 1)

// TODO: Initialize additional properties
// propertyName("propertyIdentifier", "Display Name of the Propery",
// default value (optional), minimum value (optional), maximum value (optional), increment
// (optional)); propertyIdentifier cannot have spaces
{
    // Register Ports
    addPort(meshOut);
    addPort(meshBBoxOut);
    addPort(inData);

    // Register Properties
    addProperty(propStartPoint);
    addProperty(mouseMoveStart);

    // TODO: Register additional properties
    addProperty(stepSizeEuler);
    addProperty(integrationStepsEuler);
    addProperty(stepSizeRK4);
    addProperty(integrationStepsRK4);

}

void EulerRK4Comparison::eventMoveStart(Event* event) {
    if (!inData.hasData()) return;
    auto mouseEvent = static_cast<MouseEvent*>(event);
    vec2 mousePos = mouseEvent->posNormalized();

    // Map to bounding box range
    mousePos[0] *= static_cast<float>(BBoxMax_[0] - BBoxMin_[0]);
    mousePos[1] *= static_cast<float>(BBoxMax_[1] - BBoxMin_[1]);
    mousePos += static_cast<vec2>(BBoxMin_);

    // Update starting point
    propStartPoint.set(mousePos);
    event->markAsUsed();
}

void EulerRK4Comparison::process() {
    // Get input
    if (!inData.hasData()) {
        return;
    }
    auto vol = inData.getData();

    // Retreive data in a form that we can access it
    const VectorField2 vectorField = VectorField2::createFieldFromVolume(vol);
    BBoxMin_ = vectorField.getBBoxMin();
    BBoxMax_ = vectorField.getBBoxMax();

    // The start point should be inside the volume (set maximum to the upper right corner)
    propStartPoint.setMinValue(BBoxMin_ - dvec2(1, 1));
    propStartPoint.setMaxValue(BBoxMax_ + dvec2(1, 1));

    // Initialize mesh, vertices and index buffers for the two streamlines and the points
    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;

    auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
    auto indexBufferLine = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);


    auto bboxMesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> bboxVertices;

    // Make bounding box without vertex duplication, instead of line segments which duplicate
    // vertices, create line segments between each added points with connectivity type of the index
    // buffer
    auto indexBufferBBox = bboxMesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
    // Bounding Box vertex 0
    vec4 black = vec4(0, 0, 0, 1);
    Integrator::drawNextPointInPolyline(BBoxMin_, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMin_[0], BBoxMax_[1]), black,
                                        indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(BBoxMax_, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMax_[0], BBoxMin_[1]), black,
                                        indexBufferBBox.get(), bboxVertices);
    // Connect back to the first point, to make a full rectangle
    indexBufferBBox->add(static_cast<std::uint32_t>(0));
    bboxMesh->addVertices(bboxVertices);
    meshBBoxOut.setData(bboxMesh);

    // Draw start point
    dvec2 startPoint = propStartPoint.get();
    //Integrator::drawPoint(startPoint, black, indexBufferPoints.get(), vertices);

    // TODO: Implement the Euler and Runge-Kutta of 4th order integration schemes
    // and then integrate forward for a specified number of integration steps and a given stepsize
    // (these should be additional properties of the processor)

    auto nextPointEuler = startPoint;
    float scalarvalueEuler = stepSizeEuler.get();
    int integratestepsEuler = integrationStepsEuler.get();
    vec4 red = vec4(1, 0, 0, 1);

    std::vector<vec2> eulerPoints;
    eulerPoints.push_back(nextPointEuler);
    
    Integrator::drawPoint(nextPointEuler, red, indexBufferPoints.get(), vertices);

    
    for (int i = 1; i < integratestepsEuler + 1; i++) {
        nextPointEuler = Integrator::Euler(vectorField, nextPointEuler, scalarvalueEuler);
        Integrator::drawLineSegment(vec2(eulerPoints[0][0], eulerPoints[0][1]),
                                    vec2(nextPointEuler[0], nextPointEuler[1]), red,
                                    indexBufferLine.get(), vertices);
        Integrator::drawPoint(nextPointEuler, red, indexBufferPoints.get(), vertices);
        
        eulerPoints.pop_back();
        eulerPoints.push_back(nextPointEuler);
    }

    auto nextPointRK4 = startPoint;
    float scalarvalueRK4 = stepSizeRK4.get();
    int integratestepsRK4 = integrationStepsRK4.get();
    vec4 blue = vec4(0, 0, 1, 1);
    
    std::vector<vec2> RK4Points;
    RK4Points.push_back(nextPointRK4);
    
    Integrator::drawPoint(nextPointRK4, blue, indexBufferPoints.get(), vertices);


    for (int i = 1; i < integratestepsRK4 + 1; i++) {
        nextPointRK4 = Integrator::RK4(vectorField, nextPointRK4, scalarvalueRK4);
        Integrator::drawLineSegment(vec2(RK4Points[0][0], RK4Points[0][1]),
                                    vec2(nextPointRK4[0], nextPointRK4[1]), blue,
                                    indexBufferLine.get(), vertices);
        Integrator::drawPoint(nextPointRK4, blue, indexBufferPoints.get(), vertices);
        RK4Points.pop_back();
        RK4Points.push_back(nextPointRK4);
    }



    mesh->addVertices(vertices);
    meshOut.setData(mesh);
}

}  // namespace inviwo
