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
	, propStopCondLength("stopCondLength", "Max length of streamline", 2, 0, 2)
	, propStopCondVel("stopCondVel", "Stop calculation if velocity is below", 0.05, 0, 2)
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
    vec2 nextPoint = startPoint;
    vec2 previousPoint = startPoint;
    
    vec2 BBoxMin = vectorField.getBBoxMin();
    vec2 BBoxMax = vectorField.getBBoxMax();
    
    auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
    if (displayPoints != 0){
        Integrator::drawPoint(startPoint, vec4(0, 0, 0, 1), indexBufferPoints.get(), vertices);
	}
    auto indexBufferLine = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
    
    int stepsTaken = 0;
    double arcLength = 0;
    bool stop_next = false;
    
    for (int i = 1; i < nSteps + 1; i++) {
        if(direction == 0)
            nextPoint = Integrator::RK4(vectorField, nextPoint, stepSize, false, directionField);
        else
            nextPoint = Integrator::RK4(vectorField, nextPoint, stepSize, true, directionField);
        
        if(stop_next){
            break;
        }
        if(nextPoint[0] < BBoxMin[0] || nextPoint[0] > BBoxMax[0] || nextPoint[1] < BBoxMin[1] || nextPoint[1] > BBoxMax[1]){
            nextPoint = vec2(std::clamp(nextPoint[0], BBoxMin[0], BBoxMax[0]), std::clamp(nextPoint[1], BBoxMin[1], BBoxMax[1]));
            stop_next = true;
        }
        
        arcLength += lengthVec2(previousPoint - nextPoint);
                
        if(arcLength >= maxArcLength){
            break;
        }
        if(lengthVec2(vectorField.interpolate(previousPoint)) < minSpeed){
            break;
        }
        if(lengthVec2(vectorField.interpolate(previousPoint)) == 0){
            break;
        }
        Integrator::drawLineSegment(nextPoint, previousPoint, vec4(0, 0, 0, 1),
                                    indexBufferLine.get(), vertices);
        if (displayPoints != 0)
            Integrator::drawPoint(nextPoint, vec4(0, 0, 0, 1), indexBufferPoints.get(), vertices);
        stepsTaken++;
        previousPoint = nextPoint;
    }
    return stepsTaken;
}

double StreamlineIntegrator::lengthVec2(const vec2 vec){
    return pow(vec[0]*vec[0] + vec[1]*vec[1],0.5);
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
        int stepsTaken = 0;
        if((int)propDirection.get() == 2){
            stepsTaken += DrawStreamLine(startPoint, vectorField, propStepSize.get(), 0, propDirectionField.get(),
                        propStopCondSteps.get(), propStopCondLength.get(), propStopCondVel.get(), propDisplayPoints.get(), mesh,
                        vertices);
            stepsTaken += DrawStreamLine(startPoint, vectorField, propStepSize.get(), 1, propDirectionField.get(),
                        propStopCondSteps.get(), propStopCondLength.get(), propStopCondVel.get(), propDisplayPoints.get(), mesh,
                        vertices);
        }
        else{
            stepsTaken += DrawStreamLine(startPoint, vectorField, propStepSize.get(), propDirection.get(), propDirectionField.get(),
                        propStopCondSteps.get(), propStopCondLength.get(), propStopCondVel.get(), propDisplayPoints.get(), mesh,
                        vertices);
        }
        propNumStepsTaken.set(stepsTaken);

    } else {
        // TODO: Seed multiple stream lines either randomly or using a uniform grid
        // (TODO: Bonus, sample randomly according to magnitude of the vector field)
    }

    mesh->addVertices(vertices);
    meshOut.setData(mesh);
}

}  // namespace inviwo
