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
#include <random>

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
    , propDisplayPoints("displayPoints", "Display Points", false)
    , propStartPoint("startPoint", "Start Point", vec2(0.5, 0.5), vec2(-1), vec2(1), vec2(0.1))
    , propSeedMode("seedMode", "Choose Single or Multiple Seeds")
    , propNumStepsTaken("numstepstaken", "Number of actual steps", 0, 0, 100000)
    , mouseMoveStart(
          "mouseMoveStart", "Move Start", [this](Event* e) { eventMoveStart(e); },
          MouseButton::Left, MouseState::Press | MouseState::Move)
    // TODO: Initialize additional properties
    // propertyName("propertyIdentifier", "Display Name of the Propery",
    // default value (optional), minimum value (optional), maximum value (optional),
    // increment (optional)); propertyIdentifier cannot have spaces
    , propStepSize("stepSize", "Step Size", 0.1, 0.0, 1.0, 0.05)
    , propDirection("direction", "Integration Direction")
    , propDirectional("directional", "Use Directional Field", false)
    , propStopSteps("stopSteps", "Stop at Number of Steps", 100, 0, 100000, 10)
    , propStopLength("stopLength", "Stop at Streamline Length (Disabled at 0)", 0.0, 0.0, 10.0)
    , propStopMagnitude("stopMagnitude", "Stop at Magnitude Below (Disabled at 0)", 0.0, 0.0, 10.0)
    , propColor("streamColor", "Streamline Color", vec4(0.0f, 0.0f, 0.0f, 1.0f), vec4(0.0f),
                vec4(1.0f), vec4(0.1f), InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , propSeedType("seedType", "Choose Seed Type")
    , propNumSeeds("numSeeds", "Number of Streamlines", 100, 1, 400, 10)
    , propSampleRes("sampleRes", "Sample Resolution for Magntiude Scalar Field", 100, 10, 200, 10)
    , propNumSeedsX("numSeedsX", "Number of Seed Points Along the Horizontal Axis", 10, 2, 100, 1)
    , propNumSeedsY("numSeedsY", "Number of Seed Points Along the Vertical Axis", 10, 2, 100, 1) {
    // Register Ports
    addPort(inData);
    addPort(meshOut);
    addPort(meshBBoxOut);

    // Register Properties
    addProperty(propSeedMode);
    propSeedMode.addOption("single", "Single Point Seed", 0);
    propSeedMode.addOption("multiple", "Multiple Seeds", 1);
    addProperty(propStartPoint);
    addProperty(propDisplayPoints);
    addProperty(propNumStepsTaken);
    propNumStepsTaken.setReadOnly(true);
    propNumStepsTaken.setSemantics(PropertySemantics::Text);
    addProperty(mouseMoveStart);

    // TODO: Register additional properties
    // addProperty(propertyName);

    propDirection.addOption("forward", "Forward", 0);
    propDirection.addOption("backward", "Backward", 1);

    propSeedType.addOption("random", "Uniformly Random", 0);
    propSeedType.addOption("uniform", "Uniform Grid", 1);
    propSeedType.addOption("wRandom", "Magnitude Weighted Random", 2);
    
    addProperties(propSeedType, propNumSeeds, propSampleRes, propNumSeedsX, propNumSeedsY, propStepSize, propDirection, propDirectional, propStopSteps, propStopLength,
                  propStopMagnitude, propColor);

    // Show properties for a single seed and hide properties for multiple seeds
    // (TODO)
    util::hide(propSeedType ,propNumSeeds, propSampleRes, propNumSeedsX, propNumSeedsY);
    propSeedMode.onChange([this]() {
        if (propSeedMode.get() == 0) {
            util::show(propStartPoint, mouseMoveStart, propNumStepsTaken);
            util::hide(propSeedType, propNumSeeds, propSampleRes, propNumSeedsX, propNumSeedsY);
        } else {
            util::show(propSeedType);
            util::hide(propStartPoint, mouseMoveStart, propNumStepsTaken);
            if (propSeedType.get() == 1) {
                util::show(propNumSeedsX, propNumSeedsY);
                util::hide(propNumSeeds, propSampleRes);
            } else {
                util::show(propNumSeeds, propSampleRes);
                util::hide(propNumSeedsX, propNumSeedsY);
            }
        }
    }
    );
    propSeedType.onChange([this]() {
        if (propSeedType.get() == 1) {
            util::show(propNumSeedsX, propNumSeedsY);
            util::hide(propNumSeeds);
        } else {
            util::show(propNumSeeds);
            util::hide(propNumSeedsX, propNumSeedsY);
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

    auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
    auto indexBufferLines = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);

    if (propSeedMode.get() == 0) { // Single streamline
        auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
        vec2 startPoint = propStartPoint.get();
        // Draw start point
        if (propDisplayPoints.get() != 0)
            Integrator::drawPoint(startPoint, vec4(0, 0, 0, 1), indexBufferPoints.get(), vertices);

        // TODO: Create one stream line from the given start point

        // TODO: Use the propNumStepsTaken property to show how many steps have actually been
        // integrated This could be different from the desired number of steps due to stopping
        // conditions (too slow, boundary, ...)
        int steps = Streamline(startPoint, vectorField, mesh, vertices, indexBufferPoints, indexBufferLines);
        propNumStepsTaken.set(steps);

    } else {
        // TODO: Seed multiple stream lines either randomly or using a uniform grid
        // (TODO: Bonus, sample randomly according to magnitude of the vector field)
        // Create a list of magnitudes and corresponding points for random distributions

        // Random sampling
        if (propSeedType.get() != 1) {
            // Random generator
            std::random_device rd;
            std::mt19937 gen(rd());
            int sampleRes = propSampleRes.get();
            dvec2 resolution = dvec2((BBoxMax_[0] - BBoxMin_[0]) / (double)(sampleRes),
                                     (BBoxMax_[1] - BBoxMin_[1]) / (double)(sampleRes));

            std::vector<double> magnitudeDistr; // Will contain non decreasing values from 0 to 1 corresponding to the sample points, can be viewved as a CDF
            std::vector<dvec2> points; // Contains the position of each sample point
            double sumMagnitudes = 0; // Total sum of all magnitudes
            int numSamples = 0; // Number of samples
            // Sample magnitude evenly across the volume according to resolution
            for (double x = BBoxMin_[0]; x <= BBoxMax_[0]; x += resolution[0]) {
                for (double y = BBoxMin_[1]; y <= BBoxMax_[1]; y += resolution[1]) {
                    numSamples++;
                    double magnitude = glm::length(vectorField.interpolate(dvec2(x, y)));
                    sumMagnitudes += magnitude;
                    magnitudeDistr.push_back(sumMagnitudes); // Non decreasing values of total magSums
                    points.push_back(dvec2(x, y)); // Corresponding points in vol
                }
            }
            // Uniformly Random
            if (propSeedType.get() == 0) {
                std::uniform_int_distribution<> distribution(0, numSamples - 1); // Uniform distribution accross all samples
                for (int n = 0; n < propNumSeeds.get(); n++) { // numSeeds determine number of streamlines
                    int seed = distribution(gen);
                    dvec2 startPoint = points[seed];
                    if (propDisplayPoints.get() != 0)
                        Integrator::drawPoint(startPoint, propColor.get(), indexBufferPoints.get(),
                                              vertices);
                    Streamline(startPoint, vectorField, mesh, vertices, indexBufferPoints,
                               indexBufferLines);
                }
            }
            // Magnitude Weighted Random
            else if (propSeedType.get() == 2) {
                // Normalize magnitudeDistr so we get a distribution from 0 to 1
                for (int i = 0; i < numSamples; i++) {
                    magnitudeDistr[i] /= sumMagnitudes;
                }
                std::uniform_real_distribution<> distribution(0.0, 1.0); //Uniform distr. from 0 to 1
                dvec2 startPoint = points[numSamples - 1];
                for (int n = 0; n < propNumSeeds.get(); n++) {
                    double seed = distribution(gen);
                    for (int i = 0; i < numSamples; i++) {
                        if (magnitudeDistr[i] > seed) { // Higher magnitude entries in magnitudeDistr will cover a larger span and therefore be more likely
                            startPoint = points[i];
                            break;
                        }
                    }
                    if (propDisplayPoints.get() != 0)
                        Integrator::drawPoint(startPoint, propColor.get(), indexBufferPoints.get(),
                                              vertices);
                    Streamline(startPoint, vectorField, mesh, vertices, indexBufferPoints,
                               indexBufferLines);
                }
            }
        // Uniform grid distribution
        } else {
            dvec2 resolution = dvec2((BBoxMax_[0] - BBoxMin_[0]) / (double)(propNumSeedsX.get() - 1),
                                     (BBoxMax_[1] - BBoxMin_[1]) / (double)(propNumSeedsY.get() - 1));
            // Uniform grid with nx * ny sample points
            for (double x = BBoxMin_[0]; x <= BBoxMax_[0]; x += resolution[0]) {
                for (double y = BBoxMin_[1]; y <= BBoxMax_[1]; y += resolution[1]) {
                    dvec2 startPoint = dvec2(x, y);
                    if (propDisplayPoints.get() != 0)
                        Integrator::drawPoint(startPoint, propColor.get(), indexBufferPoints.get(),
                                              vertices);
                    Streamline(startPoint, vectorField, mesh, vertices, indexBufferPoints,
                               indexBufferLines);
                }
            }
        }
    }

    mesh->addVertices(vertices);
    meshOut.setData(mesh);
}
int StreamlineIntegrator::Streamline(const dvec2& startPoint, const VectorField2& vectorField,
                                     std::shared_ptr<BasicMesh>& mesh,
                                     std::vector<BasicMesh::Vertex>& vertices,
                                     std::shared_ptr<inviwo::IndexBufferRAM>& indexBufferPoints,
                                     std::shared_ptr<inviwo::IndexBufferRAM>& indexBufferLines) {
    double step = propStepSize.get(); // Step size
    bool backwards = propDirection.get() == 1 ? true : false; // Forwards or backwards
    int stopSteps = propStopSteps.get(); // Stop after n steps
    double stopLength = propStopLength.get(); // Stop after arc length
    double stopMagnitude = propStopMagnitude.get(); // Stop at low magnitude

    dvec2 prevPoint = startPoint;
    dvec2 point = startPoint;

    int steps = 0;
    double arcLength = 0;
    bool outOfBounds = false;

    while (true) {
        point = Integrator::RK4(vectorField, prevPoint, step, backwards, propDirectional.get());
        if (!vectorField.isInside(point)) { // If out of bounds, interpolate point inbounds and stop
            point = vectorField.clampPositionToBBox(point);
            outOfBounds = true;
        }
        // Stop if moving along bbox
        if ((prevPoint[0] == BBoxMin_[0] && point[0] == BBoxMin_[0]) ||
            (prevPoint[0] == BBoxMax_[0] && point[0] == BBoxMax_[0]) ||
            (prevPoint[1] == BBoxMin_[1] && point[1] == BBoxMin_[1]) ||
            (prevPoint[1] == BBoxMax_[1] && point[1] == BBoxMax_[1]))
            break;
        if (propDisplayPoints.get()) {
            Integrator::drawPoint(point, propColor.get(), indexBufferPoints.get(),
                                  vertices);
        }
        Integrator::drawLineSegment(prevPoint, point, propColor.get(),
                                    indexBufferLines.get(), vertices);

        steps++;
        arcLength += glm::length(point - prevPoint);
        double magnitude = glm::length(vectorField.interpolate(prevPoint));

        if (steps >= stopSteps) { // Stop due to steps
            break;
        }
        if (stopLength > 0 && arcLength > stopLength) { // Stop due to arclength
            break;
        }
        if (stopMagnitude > 0 && magnitude < stopMagnitude) { // Stop due to low mag
            break;
        }
        if (magnitude < 0.001) { // Stop if mag is sufficiently close to 0
            break;
        }
        if (outOfBounds) { // Stop if out of bounds
            break;
        }

        prevPoint = point;
    }
    return steps;
}
}  // namespace inviwo
