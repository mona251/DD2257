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

    , propKernelSize("kernelSize", "Choose Kernel Size (One Diretion)", 101, 1, 1000, 10)
    , propFastLIC("fastLIC", "Use Fast LIC", true)
    , propEnhance("enhance", "Enhance Contrast", true)
    , propMean("mean", "Desired Mean", 127, 0, 255, 1)
    , propSD("sd", "Desired Standard Deviation", 50, 0, 255, 1)
// TODO: Register additional properties
{
    // Register ports
    addPort(volumeIn_);
    addPort(noiseTexIn_);
    addPort(licOut_);

    // Register properties
    // TODO: Register additional properties
    addProperties(propKernelSize, propFastLIC, propEnhance, propMean, propSD);
    
    propEnhance.onChange([this]() {
        if (propEnhance.get()) {
            util::show(propMean, propSD);
        } else {
            util::hide(propMean, propSD);
        }
    });
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

    if (propFastLIC.get()) {
        fastLIC(stepSize, visited, vectorField, texture, licImage);
    } else {
        LIC(stepSize, vectorField, texture, licImage);
    }

    if (propEnhance.get()) {
        enhance(propMean.get(), propSD.get(), licImage);
    }

    licOut_.setData(outImage);
}
void LICProcessor::LIC(double stepSize, const VectorField2& vectorField,
                       const RGBAImage& texture, RGBAImage& licImage) {
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            dvec2 position = pixelToPos(size2_t(i, j));
            std::list<dvec2> streamline =
                Integrator::Streamline(vectorField, position, stepSize, propKernelSize.get());
            int gray = 0;
            for (dvec2 point : streamline) {
                size2_t pixel = posToPixel(point);
                gray += texture.readPixelGrayScale(pixel);
            }
            gray /= streamline.size();
            licImage.setPixelGrayScale(size2_t(i, j), gray);
        }
    }
}
void LICProcessor::fastLIC(double stepSize, std::vector<std::vector<int>>& visited, const VectorField2& vectorField, const RGBAImage& texture, RGBAImage& licImage) {
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            if (visited[i][j] != 0) {
                continue;
            }
            dvec2 position = pixelToPos(size2_t(i, j));
            std::list<dvec2> streamline =
                Integrator::Streamline(vectorField, position, stepSize, propKernelSize.get());
            int gray = 0;
            for (dvec2 point : streamline) {
                size2_t pixel = posToPixel(point);
                gray += texture.readPixelGrayScale(pixel);
            }
            gray /= streamline.size();
            for (dvec2 point : streamline) {
                size2_t pixel = posToPixel(point);
                licImage.setPixelGrayScale(pixel, gray);
                visited[pixel[0]][pixel[1]] = 1;
            }
        }
    }
}
void LICProcessor::enhance(double targetMean, double targetSD, RGBAImage& licImage) {
    double mean = 0.0;
    double sumSquare = 0.0;
    double n = 0.0;
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            int gray = licImage.readPixelGrayScale(size2_t(i, j));
            if (gray == 255) continue;
            mean += gray;
            sumSquare += pow(gray, 2);
            n++;
        }
    }
    mean /= n;
    double sd = sqrt((sumSquare - n * pow(mean, 2)) / (n - 1));
    double stretch = std::min(targetSD / sd, 10.0);
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            int gray = licImage.readPixelGrayScale(size2_t(i, j));
            gray = int(targetMean + stretch * (gray - mean));
            licImage.setPixelGrayScale(size2_t(i, j), gray);
        }
    }
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
