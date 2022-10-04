/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Monday, October 02, 2017 - 13:31:17
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/datastructures/volume/volumeram.h>
#include <lablic/licprocessor.h>
#include <labstreamlines/integrator.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo LICProcessor::processorInfo_{
    "org.inviwo.LICProcessor",  // Class identifier
    "LICProcessor",             // Display name
    "KTH Labs",                 // Category
    CodeState::Experimental,    // Code state
    Tags::None,                 // Tags
};

const ProcessorInfo LICProcessor::getProcessorInfo() const { return processorInfo_; }

LICProcessor::LICProcessor()
    : Processor()
    , volumeIn_("volIn")
    , noiseTexIn_("noiseTexIn")
    , licOut_("licOut")
// TODO: Register additional properties
    , propKernelSize("kernelSize", "Kernel Size", 50, 1, 100, 1)
    , propSkipNPoints("skipNpoints", "Skip N Points", 0, 0, 10, 1)
    , propStepSize("stepSize", "Step Size", 0.001, 0, 0.1, 0.001)
{
    // Register ports
    addPort(volumeIn_);
    addPort(noiseTexIn_);
    addPort(licOut_);

    // Register properties
    // TODO: Register additional properties
    addProperty(propKernelSize);
    addProperty(propSkipNPoints);
    addProperty(propStepSize);
    
}
std::vector<dvec2>& LICProcessor::streamLineLIC(const VectorField2& vectorField, int kernelSize, const dvec2& position, double step, int skipNPoints){
    std::vector<dvec2> points;
    dvec2 startPos = position;
    dvec2 currentPos = position;
    dvec2 previousPos = position;
    
    points.push_back(position);
    int N_points = kernelSize*(skipNPoints + 1);
    
    bool outOfBounds = false;
    
    for(int i = 1; i<=N_points; i++){
        currentPos = Integrator::RK4(vectorField, previousPos, step, false, true);
        
        double magnitude = glm::length(vectorField.interpolate(previousPos));
        
        if (!vectorField.isInside(currentPos)) { // If out of bounds, interpolate point inbounds and stop
            currentPos = vectorField.clampPositionToBBox(currentPos);
            outOfBounds = true;
        }
        if (magnitude < 0.001) { // Stop if mag is sufficiently close to 0
            break;
        }
        if (outOfBounds) { // Stop if out of bounds
            break;
        }
        
        if(i % (skipNPoints + 1) == 0){
            points.push_back(currentPos);
        }      
        previousPos = currentPos;
    }
    
    outOfBounds = false;
    
    for(int i = 1; i<=N_points; i++){
        currentPos = Integrator::RK4(vectorField, previousPos, step, true, true);
        
        double magnitude = glm::length(vectorField.interpolate(previousPos));
        
        if (!vectorField.isInside(currentPos)) { // If out of bounds, interpolate point inbounds and stop
            currentPos = vectorField.clampPositionToBBox(currentPos);
            outOfBounds = true;
        }
        if (magnitude < 0.001) { // Stop if mag is sufficiently close to 0
            break;
        }
        if (outOfBounds) { // Stop if out of bounds
            break;
        }
        
        if(i % (skipNPoints + 1) == 0){
            points.push_back(currentPos);
        }
        previousPos = currentPos;
    }
    return points;
}

void LICProcessor::LIC(RGBAImage& licImage, const RGBAImage& texture, const VectorField2& vectorField, int kernelSize, double step, int skipNPoints){
    dvec2 BBoxMin = vectorField.getBBoxMin();
    dvec2 BBoxMax = vectorField.getBBoxMax();
    
    for (size_t j = 0; j < LICProcessor::texDims_.y; j++) {
        for (size_t i = 0; i < LICProcessor::texDims_.x; i++) {
            dvec2 startPos_VectorSpace = dvec2(BBoxMin[0] + (double)i/(double)LICProcessor::texDims_.x*(BBoxMax[0] - BBoxMin[0]),
                                               BBoxMin[1] + (double)j/(double)LICProcessor::texDims_.y*(BBoxMax[1] - BBoxMin[1]));
            std::vector<dvec2> points = streamLineLIC(vectorField,
                    kernelSize, startPos_VectorSpace, step, skipNPoints);
            int Nsamples = static_cast<int>(points.size());
            double sum = 0;
            for(int k = 0; k<Nsamples;k++){
                ivec2 samplePos_TextureSpace = ivec2((points[k][0] - BBoxMin[0])*(double)LICProcessor::texDims_.x/(BBoxMax[0] - BBoxMin[0]),
                                                     (points[k][1] - BBoxMin[1])*(double)LICProcessor::texDims_.y/(BBoxMax[1] - BBoxMin[1]));
                sum += texture.readPixelGrayScale(samplePos_TextureSpace)/Nsamples;
            }
            licImage.setPixelGrayScale(size2_t(i, j), sum);
        }
    }
}

void LICProcessor::process() {
    // Get input
    if (!volumeIn_.hasData()) {
        return;
    }

    if (!noiseTexIn_.hasData()) {
        return;
    }

    auto vol = volumeIn_.getData();
    const VectorField2 vectorField = VectorField2::createFieldFromVolume(vol);
    vectorFieldDims_ = vol->getDimensions();

    auto tex = noiseTexIn_.getData();
    const RGBAImage texture = RGBAImage::createFromImage(tex);
    texDims_ = tex->getDimensions();

    double value = texture.readPixelGrayScale(size2_t(0, 0));

    //LogProcessorInfo(value);

    // Prepare the output, it has the same dimensions as the texture and rgba values in [0,255]
    auto outImage = std::make_shared<Image>(texDims_, DataVec4UInt8::get());
    RGBAImage licImage(outImage);

    std::vector<std::vector<double>> licTexture(texDims_.x, std::vector<double>(texDims_.y, 0.0));

    // Hint: Output an image showing which pixels you have visited for debugging
    std::vector<std::vector<int>> visited(texDims_.x, std::vector<int>(texDims_.y, 0));
    
    LIC(licImage, texture, vectorField, propKernelSize.get(), propStepSize.get(), propSkipNPoints.get());
    
    // TODO: Implement LIC and FastLIC
    // This code instead sets all pixels to the same gray value
    
    
    

    licOut_.setData(outImage);
}

}  // namespace inviwo
