/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Tuesday, September 19, 2017 - 15:08:33
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/interaction/events/mouseevent.h>
#include <inviwo/core/util/utilities.h>
#include <labstreamlines/integrator.h>
#include <labstreamlines/streamlineintegrator.h>
#include <labutils/scalarvectorfield.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming
// scheme
const ProcessorInfo StreamlineIntegrator::processorInfo_{
    "org.inviwo.StreamlineIntegrator",  // Class identifier
    "Streamline Integrator",            // Display name
    "KTH Lab",                          // Category
    CodeState::Experimental,            // Code state
    Tags::None,                         // Tags
};

const ProcessorInfo StreamlineIntegrator::getProcessorInfo() const { return processorInfo_; }

StreamlineIntegrator::StreamlineIntegrator()
    : Processor()
    , inData("volIn")
    , meshOut("meshOut")
    , meshBBoxOut("meshBBoxOut")
    , propDisplayPoints("displayPoints", "Display Points", true)
    , propStartPoint("startPoint", "Start Point", vec2(0.5, 0.5), vec2(-1), vec2(1), vec2(0.1))
    , propSeedMode("seedMode", "Seeds")
    , propDirection("direction", "Direction")
    , propNumStepsTaken("numstepstaken", "Number of actual steps", 0, 0, 100000)
    , propStepSize("stepSize", "Step size", 0.1, 0.0, 1.0, 0.001)
    , propDirectionField("directionField", "Direction Field Integration", false)
    , propStopCondSteps("stopCondSteps", "Max number of steps", 10, 0, 1000)
    , propStopCondLength("stopCondLength", "Max length of streamline", 1000, 0, 100000)
    , propStopCondVel("stopCondVel", "Stop calculation if velocity is below", 1000, 0, 100000)
    , mouseMoveStart(
          "mouseMoveStart", "Move Start", [this](Event* e) { eventMoveStart(e); },
          MouseButton::Left, MouseState::Press | MouseState::Move)
// TODO: Initialize additional properties
// propertyName("propertyIdentifier", "Display Name of the Propery",
// default value (optional), minimum value (optional), maximum value (optional),
// increment (optional)); propertyIdentifier cannot have spaces
{
    // Register Ports
    addPort(inData);
    addPort(meshOut);
    addPort(meshBBoxOut);

    // Register Properties
    propSeedMode.addOption("one", "Single Start Point", 0);
    propSeedMode.addOption("multiple", "Multiple Seeds", 1);
    addProperty(propSeedMode);
    addProperty(propStartPoint);
    addProperty(propDisplayPoints);
    addProperty(propNumStepsTaken);
    propNumStepsTaken.setReadOnly(true);
    propNumStepsTaken.setSemantics(PropertySemantics::Text);
    addProperty(mouseMoveStart);

    // TODO: Register additional properties
    // addProperty(propertyName);
    addProperty(propDirection);
    propDirection.addOption("forward", "Forward", 0);
    propDirection.addOption("backward", "Backward", 1);
    propDirection.addOption("backward", "Both", 2);

    addProperty(propStepSize);
    addProperty(propDirectionField);

    addProperty(propStopCondSteps);
    addProperty(propStopCondLength);
    addProperty(propStopCondVel);

	

    // Show properties for a single seed and hide properties for multiple seeds
    // (TODO)
    propSeedMode.onChange([this]() {
        if (propSeedMode.get() == 0) {
            util::show(propStartPoint, mouseMoveStart, propNumStepsTaken);
            // util::hide(...)
        } else {
            util::hide(propStartPoint, mouseMoveStart, propNumStepsTaken);
            // util::show(...)
        }
    });
}

void StreamlineIntegrator::eventMoveStart(Event* event) {
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

int StreamlineIntegrator::DrawStreamLine(const vec2& startPoint, const VectorField2& vectorField,
                        const float stepSize, const int direction, const bool directionField,
                        const int nSteps, const float maxArcLength, const float minSpeed, const bool displayPoints,
                        std::shared_ptr<BasicMesh>& mesh, std::vector<BasicMesh::Vertex>& vertices){
    vec2 nextPointRK4 = startPoint;
    std::vector<vec2> RK4Points;
    RK4Points.push_back(nextPointRK4);
    
    auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
    if (displayPoints != 0){
        Integrator::drawPoint(startPoint, vec4(0, 0, 0, 1), indexBufferPoints.get(), vertices);
	}
    auto indexBufferLine = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
    
    int stepsTaken = 0;
    for (int i = 1; i < nSteps + 1; i++) {
        if(false){
            //Stop condition
        }
        stepsTaken++;
        nextPointRK4 = Integrator::RK4(vectorField, nextPointRK4, stepSize);
        Integrator::drawLineSegment(vec2(RK4Points[0][0], RK4Points[0][1]),
                                    vec2(nextPointRK4[0], nextPointRK4[1]), vec4(0, 0, 0, 1),
                                    indexBufferLine.get(), vertices);
        if (displayPoints != 0)
            Integrator::drawPoint(nextPointRK4, vec4(0, 0, 0, 1), indexBufferPoints.get(), vertices);
        RK4Points.pop_back();
        RK4Points.push_back(nextPointRK4);
    }
    return stepsTaken;
}

void StreamlineIntegrator::process() {
    // Get input
    if (!inData.hasData()) {
        return;
    }
    auto vol = inData.getData();

    // Retreive data in a form that we can access it
    auto vectorField = VectorField2::createFieldFromVolume(vol);
    BBoxMin_ = vectorField.getBBoxMin();
    BBoxMax_ = vectorField.getBBoxMax();

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

    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;

    if (propSeedMode.get() == 0) {
        auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
        vec2 startPoint = propStartPoint.get();
        // Draw start point
        int stepsTaken = DrawStreamLine(startPoint, vectorField, propStepSize.get(), propDirection.get(), propDirectionField.get(),
                        propStopCondSteps.get(), propStopCondLength.get(), propStopCondVel.get(), propDisplayPoints.get(), mesh,
                        vertices);
        propNumStepsTaken.set(stepsTaken);

    } else {
        // TODO: Seed multiple stream lines either randomly or using a uniform grid
        // (TODO: Bonus, sample randomly according to magnitude of the vector field)
    }

    mesh->addVertices(vertices);
    meshOut.setData(mesh);
}

}  // namespace inviwo
