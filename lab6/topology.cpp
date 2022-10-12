/*********************************************************************
 *  Author  : Anke Friederici
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 **********************************************************************/
/*********************************************************************
    6.2 Svar:
    i origo:
    center: ex. y,-x ger rotation. flytta med konstanter för att få noll i ösnkad pkt
    sadel: ex. -x,y ger fält mot nollpkt längs med axlar. flytta med konst. för att få 0 i önskad pkt
    båda: center.Xsadel.X, center.Ysadel.Y ger bäggre noll pktr
    skalär-fält topologi = vektor-fält topologi av skalär-fält gradienten
    a) 3+y, 5-x
    b) -2-x, -7+y
    c) (3+y)(-2-x), (5-x)(-7+y)
    d) cos(x), -sin(y)
 **********************************************************************/

#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <labstreamlines/integrator.h>
#include <labutils/scalarvectorfield.h>
#include <labtopo/topology.h>
#include <labtopo/utils/gradients.h>

namespace inviwo {

const vec4 Topology::ColorsCP[6] = {
    vec4(1, 1, 0, 1),    // Saddle - Yellow
    vec4(1, 0, 0, 1),    // AttractingNode - Red
    vec4(0, 0, 1, 1),    // RepellingNode - Blue
    vec4(0.5, 0, 1, 1),  // AttractingFocus - Purple
    vec4(1, 0.5, 0, 1),  // RepellingFocus - Orange
    vec4(0, 1, 0, 1)     // Center - Green
};

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo Topology::processorInfo_{
    "org.inviwo.Topology",    // Class identifier
    "Vector Field Topology",  // Display name
    "KTH Lab",                // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};

const ProcessorInfo Topology::getProcessorInfo() const { return processorInfo_; }

Topology::Topology()
    : Processor()
    , inData("inData")
    , outMesh("meshOut")
    , meshBBoxOut("meshBBoxOut")
    // TODO: Initialize additional properties
    // propertyName("propertyIdentifier", "Display Name of the Propery",
    // default value (optional), minimum value (optional), maximum value (optional), increment
    // (optional)); propertyIdentifier cannot have spaces
    , propEpsilon("epsilon", "Decomposition Min Diagonal Fraction", 0.01, 0.000000001, 1.0, 0.0001)
    , propCenter("center", "Center Threshold for Classification", 0.00005, 0.0, 1.0, 0.00005)
    , propSeparatrices("separatrices", "Show Separatrices", true)
    , propStep("step", "Streamline Step Size", 0.01, 0.0, 1.0, 0.001)
    , propSteps("steps", "Strealine Max Steps", 1000, 1, 100000, 10)
    , propBoundarySwitch("bs", "Show Boundary Switch Points", true)

{
    // Register Ports
    addPort(outMesh);
    addPort(inData);
    addPort(meshBBoxOut);

    // TODO: Register additional properties
    // addProperty(propertyName);
    addProperties(propEpsilon, propCenter, propSeparatrices, propStep, propSteps, propBoundarySwitch);

    propSeparatrices.onChange([this]() {
    if (propSeparatrices.get()) {
        util::show(propStep, propSteps);
    } else {
        util::hide(propStep, propSteps);
    }
    });

}

void Topology::process() {
    // Get input
    if (!inData.hasData()) {
        return;
    }
    auto vol = inData.getData();

    // Retreive data in a form that we can access it
    const VectorField2 vectorField = VectorField2::createFieldFromVolume(vol);

    // Add a bounding box to the mesh
    const dvec2& BBoxMin = vectorField.getBBoxMin();
    const dvec2& BBoxMax = vectorField.getBBoxMax();
    auto bboxMesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> bboxVertices;
    auto indexBufferBBox = bboxMesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
    // Bounding Box vertex 0
    vec4 black = vec4(0, 0, 0, 1);
    Integrator::drawNextPointInPolyline(BBoxMin, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMin[0], BBoxMax[1]), black, indexBufferBBox.get(),
                                        bboxVertices);
    Integrator::drawNextPointInPolyline(BBoxMax, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMax[0], BBoxMin[1]), black, indexBufferBBox.get(),
                                        bboxVertices);
    // Connect back to the first point, to make a full rectangle
    indexBufferBBox->add(static_cast<std::uint32_t>(0));
    bboxMesh->addVertices(bboxVertices);
    meshBBoxOut.setData(bboxMesh);

    // Initialize mesh, vertices and index buffers for seperatrices
    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;
    // Either add all line segments to this index buffer (one large buffer, two consecutive points
    // make up one line), or use several index buffers with connectivity type strip.
    auto indexBufferSeparatrices = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
    // auto indexBufferSeparatrices = mesh->addIndexBuffer(DrawType::Lines,
    // ConnectivityType::Strip);

    auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);

    // TODO: Compute the topological skeleton of the input vector field.
    // Find the critical points and color them according to their type.
    // Integrate all separatrices.

    dims = vectorField.getNumVerticesPerDim();
    diagonal = glm::length(vectorField.getCellSize());

    // Other helpful functions
    // dvec2 pos = vectorField.getPositionAtVertex(size2_t(i, j));
    // Computing the jacobian at a position
    // dmat2 jacobian = vectorField.derive(pos);
    // Doing the eigen analysis
    // auto eigenResult = util::eigenAnalysis(jacobian);
    // The result of the eigen analysis has attributed eigenvaluesRe eigenvaluesIm and
    // eigenvectors

    // Accessing the colors
    vec4 colors[6] = {ColorsCP[static_cast<int>(TypeCP::Saddle)],
                      ColorsCP[static_cast<int>(TypeCP::RepellingNode)],
                      ColorsCP[static_cast<int>(TypeCP::AttractingNode)],
                      ColorsCP[static_cast<int>(TypeCP::Center)],
                      ColorsCP[static_cast<int>(TypeCP::AttractingFocus)],
                      ColorsCP[static_cast<int>(TypeCP::RepellingFocus)]};

    std::list<dvec2> cps = critPoints(vectorField);
    for (dvec2 cp : cps) {
        classify(cp, vectorField, indexBufferPoints, indexBufferSeparatrices, vertices, colors);
    }
    if (propBoundarySwitch.get()) {
        std::list<dvec2> bsps = boundarySwitchPoints(vectorField);
        for (dvec2 bsp : bsps) {
            drawBorderPoint(bsp, propStep.get(), propSteps.get(), vectorField, indexBufferPoints,
                            indexBufferSeparatrices, vertices);
        }
    }

    mesh->addVertices(vertices);
    outMesh.setData(mesh);
}
void Topology::classify(dvec2 critPoint, const VectorField2& vectorField,
                        std::shared_ptr<inviwo::IndexBufferRAM>& indexBufferPoints,
                        std::shared_ptr<inviwo::IndexBufferRAM>& indexBufferSeparatrices,
                        std::vector<BasicMesh::Vertex>& vertices, const vec4 colors[6]) {
    // J = jacobian
    // D = det(J) = determinant of jacobian
    // T = sum(diagonal(J)) = trace of jacobian
    glm::highp_dmat2 J = vectorField.derive(critPoint);
    double D = J[0][0] * J[1][1] - J[1][0] * J[0][1];
    if (D == 0) {
        return;
    }
    util::EigenResult eigenResults = util::eigenAnalysis(J);
    double R1 = eigenResults.eigenvaluesRe[0];
    double R2 = eigenResults.eigenvaluesRe[1];
    double I1 = eigenResults.eigenvaluesIm[0];
    double I2 = eigenResults.eigenvaluesIm[1];
    // No imaginary part: Saddle, Repelling node, Attracting node
    if (I1 == 0 && I2 == 0) {
        // Saddle
        if (R1 * R2 < 0) {
            Integrator::drawPoint(critPoint, colors[0], indexBufferPoints.get(), vertices);
            if (propSeparatrices.get()) {
                separatrices(critPoint, eigenResults.eigenvectors, propStep.get(), propSteps.get(),
                             vectorField, vec4(1, 1, 1, 1), indexBufferSeparatrices, vertices);
            }
            return;
        }
        // Repelling node
        if (R1 > 0 && R2 > 0) {
            Integrator::drawPoint(critPoint, colors[1], indexBufferPoints.get(), vertices);
            return;
        }
        // Attracting node
        if (R1 < 0 && R2 < 0) {
            Integrator::drawPoint(critPoint, colors[2], indexBufferPoints.get(), vertices);
            return;
        }
    }
    // Center, Attracting focus, Repelling focus
    else {
        // Center
        if (abs(R1) <= propCenter.get() && abs(R2) <= propCenter.get() && I1 == -I2) {
            Integrator::drawPoint(critPoint, colors[3], indexBufferPoints.get(), vertices);
            return;
        }
        // Attracting focus
        if (R1 == R2 && R1 < 0 && I1 == -I2) {
            Integrator::drawPoint(critPoint, colors[4], indexBufferPoints.get(), vertices);
            return;
        }
        // Repelling focus
        if (R1 == R2 && R1 > 0 && I1 == -I2) {
            Integrator::drawPoint(critPoint, colors[5], indexBufferPoints.get(), vertices);
            return;
        }
    }
}

std::list<dvec2> Topology::critPoints(const VectorField2& vectorField) {
    std::list<dvec2> cps;
    // Looping through all values in the vector field.
    for (size_t j = 0; j < dims[1] - 1; ++j) {
        for (size_t i = 0; i < dims[0] - 1; ++i) {
            dvec2 v1 = vectorField.getValueAtVertex(size2_t(i, j));
            dvec2 v2 = vectorField.getValueAtVertex(size2_t(i + 1, j));
            dvec2 v3 = vectorField.getValueAtVertex(size2_t(i, j + 1));
            dvec2 v4 = vectorField.getValueAtVertex(size2_t(i + 1, j + 1));
            dvec2 pos = vectorField.getPositionAtVertex(size2_t(i, j));

            dvec2 critPoint =
                decomposition(v1, v2, v3, v4, vectorField.getCellSize(), pos, vectorField);
            if (!isnan(critPoint.x)) {
                cps.push_back(critPoint);
            }
        }
    }
    return cps;
}

dvec2 Topology::decomposition(const dvec2& v1, const dvec2& v2, const dvec2& v3, const dvec2& v4, dvec2 spacing,
                              dvec2 position, const VectorField2& vectorField) {
    // v3 v34 v4
    // v13[v14]v24
    // v1 v12 v2
    // Returns nan,nan vector if no crit-point is found in sector
    // 
    // Check if the input vectors are crit points
    dvec2 positions[6] = {position, position + dvec2(spacing.x * 0.5, 0),
                          position + dvec2(0, spacing.y * 0.5), position + spacing * 0.5,
                          position + dvec2(spacing.x, spacing.y * 0.5), position + dvec2(spacing.x * 0.5, spacing.y)};
    if (isolated(v1, position, vectorField)) return position;
    if (isolated(v2, positions[1], vectorField)) return positions[1];
    if (isolated(v3, positions[2], vectorField)) return positions[2];
    if (isolated(v4, positions[3], vectorField)) return positions[3];
    // Run decomposition
    if (signTest(v1, v2, v3, v4)) {
        if (glm::length(spacing) <= diagonal * propEpsilon.get()) {
            return position + spacing * 0.5;
        } else {
            const dvec2 v12 = vectorField.interpolate(positions[1]);
            const dvec2 v13 = vectorField.interpolate(positions[2]);
            const dvec2 v14 = vectorField.interpolate(positions[3]);
            const dvec2 v24 = vectorField.interpolate(positions[4]);
            const dvec2 v34 = vectorField.interpolate(positions[5]);
            dvec2 quadrants[4][4] = {
                {v1, v12, v13, v14}, {v12, v2, v14, v24}, {v13, v14, v3, v34}, {v14, v24, v34, v4}};
            for (int q = 0; q < 4; q++) {
                dvec2 critPoint =
                    decomposition(quadrants[q][0], quadrants[q][1], quadrants[q][2],
                                  quadrants[q][3], spacing * 0.5, positions[q], vectorField);
                if (!isnan(critPoint.x)) {
                    return critPoint;
                }
            }
        }
    }
    return dvec2(NAN, NAN);
}

bool Topology::signTest(const dvec2& v1, const dvec2& v2, const dvec2& v3, const dvec2& v4) {
    for (int i = 0; i < 2; i++){
        if ((v1[i] > 0 && v2[i] > 0 && v3[i] > 0 && v4[i] > 0) ||
            (v1[i] < 0 && v2[i] < 0 && v3[i] < 0 && v4[i] < 0)) {
            return false;
        }
    }
    return true;
}

bool Topology::isolated(const dvec2& v, dvec2& pos, const VectorField2& vectorField) {
    if (glm::length(v) != 0) {
        return false;
    }
    dvec2 epsilon = vectorField.getCellSize() * propEpsilon.get();
    for (int x = -1; x < 2; x += 2) {
        for (int y = -1; y < 2; y += 2) {
            if (glm::length(vectorField.interpolate(pos + dvec2(epsilon[0] * x, epsilon[1] * y))) == 0)
                return false;
        }
    }
    return true;
}

void Topology::separatrices(dvec2 critPoint, mat2& eigenVectors, double step,
                           double steps, const VectorField2& vectorField, const vec4& color,
                           std::shared_ptr<inviwo::IndexBufferRAM>& indexBufferSeparatrices,
                           std::vector<BasicMesh::Vertex>& vertices) {
    for (int i = 0; i < 2; i++) {
        for (int dir = -1; dir < 2; dir += 2) {
            dvec2 e = eigenVectors[i];
            if (!vectorField.isInside(critPoint + dir * step * e)) {
                continue;
            }
            std::list<dvec2> separatix =
                Integrator::Streamline(vectorField, critPoint + dir * step * e, step, steps);
            dvec2 prevPoint = separatix.front();
            separatix.pop_front();
            for (dvec2 point : separatix) {
                Topology::drawLineSegment(prevPoint, point, color, indexBufferSeparatrices.get(),
                                            vertices);
                prevPoint = point;
            }
        }
    }
}
std::list<dvec2> Topology::boundarySwitchPoints(const VectorField2& vectorField) {
    std::list<dvec2> bsps;
    // Looping through horizontal vectorfield borders
    for (size_t j = 0; j < dims[1]; j += dims[1]-1) {
        for (size_t i = 0; i < dims[0] - 1; ++i) {
            dvec2 v1 = vectorField.getValueAtVertex(size2_t(i, j));
            dvec2 v2 = vectorField.getValueAtVertex(size2_t(i + 1, j));
            dvec2 pos = vectorField.getPositionAtVertex(size2_t(i, j));

            dvec2 bsPoint =
                borderDecomposition(v1, v2, vectorField.getCellSize() * dvec2(1, 0), pos, vectorField);
            if (!isnan(bsPoint.x)) {
                bsps.push_back(bsPoint);
            }
        }
    }
    // Looping through vertical vectorfield borders
    for (size_t j = 0; j < dims[1] - 1; j++) {
        for (size_t i = 0; i < dims[0]; i += dims[0] - 1) {
            dvec2 v1 = vectorField.getValueAtVertex(size2_t(i, j));
            dvec2 v2 = vectorField.getValueAtVertex(size2_t(i, j + 1));
            dvec2 pos = vectorField.getPositionAtVertex(size2_t(i, j));

            dvec2 bsPoint = borderDecomposition(v1, v2, vectorField.getCellSize() * dvec2(0, 1),
                                                pos, vectorField);
            if (!isnan(bsPoint.x)) {
                bsps.push_back(bsPoint);
            }
        }
    }
    return bsps;
}
dvec2 Topology::borderDecomposition(const dvec2& v1, const dvec2& v2,  dvec2 spacing, dvec2 pos,
    const VectorField2& vectorField) {
    // v1 v12 v2
    // Returns nan,nan vector if no boundary switch point is found in sector
    // Dir variables
    int along = spacing[0] > 0 ? 0 : 1;
    int cross = 1 - along;
    // Run decomposition
    if (v1[cross] * v2[cross] < 0) {
        if (spacing[along] <= vectorField.getCellSize()[along] * propEpsilon.get()) {
            return pos + spacing * 0.5;
        } else {
            const dvec2 v12 = vectorField.interpolate(pos + spacing * 0.5);
            dvec2 positions[2] = {pos, pos + spacing * 0.5};
            dvec2 sections[2][2] = {
                {v1, v12}, {v12, v2}};
            for (int s = 0; s < 2; s++) {
                dvec2 bsPoint = borderDecomposition(sections[s][0], sections[s][1], spacing * 0.5,
                                                      positions[s], vectorField);
                if (!isnan(bsPoint.x)) {
                    return bsPoint;
                }
            }
        }
    }
    return dvec2(NAN, NAN);
}
void Topology::drawBorderPoint(dvec2 bsPoint, double step, double steps, const VectorField2& vectorField,
                               std::shared_ptr<inviwo::IndexBufferRAM>& indexBufferPoints,
                               std::shared_ptr<inviwo::IndexBufferRAM>& indexBufferSeparatrices,
                               std::vector<BasicMesh::Vertex>& vertices) {
    glm::highp_dmat2 J = vectorField.derive(bsPoint);
    double D = J[0][0] * J[1][1] - J[1][0] * J[0][1];
    if (D == 0) {
        return;
    }
    util::EigenResult eigenResults = util::eigenAnalysis(J);
    Integrator::drawPoint(bsPoint, vec4(0.5, 0.5, 0.5, 1), indexBufferPoints.get(), vertices);
    if (propSeparatrices.get()) {
        separatrices(bsPoint, eigenResults.eigenvectors, step, steps, vectorField,
                     vec4(0.5, 0.5, 0.5, 1), indexBufferSeparatrices, vertices);
    }


}
void Topology::drawLineSegment(const dvec2& v1, const dvec2& v2, const vec4& color,
                               IndexBufferRAM* indexBuffer,
                               std::vector<BasicMesh::Vertex>& vertices) {
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v1[0], v1[1], 0), vec3(0, 0, 1), vec3(v1[0], v1[1], 0), color});
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v2[0], v2[1], 0), vec3(0, 0, 1), vec3(v2[0], v2[1], 0), color});
}

}  // namespace inviwo
