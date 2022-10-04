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

    , propKernelSize("kernelSize", "Choose Kernel Size (One Diretion)", 10, 1, 100, 1)
// TODO: Register additional properties
{
    // Register ports
    addPort(volumeIn_);
    addPort(noiseTexIn_);
    addPort(licOut_);

    // Register properties
    // TODO: Register additional properties
    addProperties(propKernelSize);
}

void LICProcessor::LIC(RGBAImage& licImage, const RGBAImage& texture, const VectorField2& vectorField, int kernelSize, double stepSize){
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            dvec2 position = LICProcessor::pixelToPos(size2_t(i, j));
            std::list<dvec2> streamline =
                Integrator::Streamline(vectorField, position, stepSize, propKernelSize.get());
            int sum = 0;
            for (dvec2 point : streamline) {
                size2_t pixel = LICProcessor::posToPixel(point);
                sum += texture.readPixelGrayScale(pixel);
            }
            sum /= streamline.size();
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

    LogProcessorInfo(value);

    // Prepare the output, it has the same dimensions as the texture and rgba values in [0,255]
    auto outImage = std::make_shared<Image>(texDims_, DataVec4UInt8::get());
    RGBAImage licImage(outImage);

    std::vector<std::vector<double>> licTexture(texDims_.x, std::vector<double>(texDims_.y, 0.0));

    // Hint: Output an image showing which pixels you have visited for debugging
    std::vector<std::vector<int>> visited(texDims_.x, std::vector<int>(texDims_.y, 0));

    // TODO: Implement LIC and FastLIC
    // This code instead sets all pixels to the same gray value
    bboxMin = vectorField.getBBoxMin();
    bboxMax = vectorField.getBBoxMax();
    pixelSize = dvec2((bboxMax[0] - bboxMin[0]) / (double)texDims_[0],
                                  (bboxMax[1] - bboxMin[1]) / (double)texDims_[1]);
    double stepSize = std::min(pixelSize[0], pixelSize[1]);

    LIC(licImage, texture, vectorField, propKernelSize.get(), stepSize);

    licOut_.setData(outImage);
}
dvec2 LICProcessor::pixelToPos(size2_t pixel) {
    return dvec2(bboxMin[0] + ((double)pixel[0] + 0.5) * pixelSize[0],
                 bboxMin[1] + ((double)pixel[1] + 0.5) * pixelSize[1]);
}
size2_t LICProcessor::posToPixel(dvec2 pos) {
    return size2_t(std::min((int)floor((pos[0] - bboxMin[0]) / pixelSize[0]), (int)texDims_[0] - 1),
                   std::min((int)floor((pos[1] - bboxMin[1]) / pixelSize[1]), (int)texDims_[1] - 1));
}
}  // namespace inviwo
